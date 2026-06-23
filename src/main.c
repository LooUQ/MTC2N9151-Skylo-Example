/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <zephyr/shell/shell.h>

#include <date_time.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <modem/ntn.h>
#include <nrf_modem_at.h>

K_SEM_DEFINE(lte_connected, 0, 1);

/* UDP socket, kept open after the initial send so messages can be sent
 * manually with the "udp send <text>" shell command. -1 until registered.
 */
static int udp_fd = -1;

/* Running field-test tallies. cycles counts periodic loop iterations; good
 * sends/receives count successful send()/recv() calls (including manual
 * "udp send", which also flows through udp_send_and_recv()).
 */
static unsigned int total_cycles;
static unsigned int good_sends;
static unsigned int good_recvs;

/* log_tally() reads the time, but read_unix_time() is defined later with the
 * other modem helpers. */
static int64_t read_unix_time(void);

/* Advance past any characters that cannot start a number (quotes, commas,
 * spaces), so the CONFIG_NTN_LOCATION string can be tokenized with strtod
 * regardless of whether the values are quoted.
 */
static const char *skip_to_number(const char *p)
{
	while (*p && *p != '+' && *p != '-' && *p != '.' &&
	       (*p < '0' || *p > '9')) {
		p++;
	}
	return p;
}

/* Parse "lat,lon,alt" (as configured in CONFIG_NTN_LOCATION) into numeric
 * values for ntn_location_set(). Returns 0 on success, -1 on parse error.
 */
static int parse_location(const char *str, double *lat, double *lon, float *alt)
{
	char *end;
	const char *p = skip_to_number(str);

	*lat = strtod(p, &end);
	if (end == p) {
		return -1;
	}

	p = skip_to_number(end);
	*lon = strtod(p, &end);
	if (end == p) {
		return -1;
	}

	p = skip_to_number(end);
	*alt = (float)strtod(p, &end);
	if (end == p) {
		return -1;
	}

	return 0;
}

static void ntn_handler(const struct ntn_evt *evt)
{
	double lat, lon;
	float alt;
	int err;

	switch (evt->type) {
	case NTN_EVT_LOCATION_REQUEST:
		if (!evt->location_request.requested) {
			break;
		}

		printk("NTN location requested (accuracy %u m)\n",
		       evt->location_request.accuracy);

		/* NTN requires the modem to know the device position for
		 * Doppler/timing pre-compensation. A stationary device can supply
		 * a single fixed fix (from CONFIG_NTN_LOCATION); a mobile device
		 * would feed live GNSS fixes here instead.
		 */
		if (parse_location(CONFIG_NTN_LOCATION, &lat, &lon, &alt) != 0) {
			printk("Invalid CONFIG_NTN_LOCATION: \"%s\"\n",
			       CONFIG_NTN_LOCATION);
			break;
		}

		/* validity 0 = never expires; valid for a stationary device. */
		err = ntn_location_set(lat, lon, alt, 0);
		if (err) {
			printk("ntn_location_set failed, error: %d\n", err);
		}
		break;
	default:
		printk("Unknown NTN event: %d\n", evt->type);
		break;
	}
}

static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
		    (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			/* Satellite registration can take minutes and pass through
			 * several intermediate states. Distinguish the transient
			 * "still searching" states from the ones the network rejects
			 * us with, so a stuck attach is obvious instead of looking
			 * like it is merely slow.
			 */
			const char *reason;

			switch (evt->nw_reg_status) {
			case LTE_LC_NW_REG_SEARCHING:
				reason = "searching";
				break;
			case LTE_LC_NW_REG_REGISTRATION_DENIED:
				reason = "DENIED by network";
				break;
			case LTE_LC_NW_REG_NO_SUITABLE_CELL:
				reason = "no suitable cell (PLMN/cell rejected)";
				break;
			case LTE_LC_NW_REG_UICC_FAIL:
				reason = "UICC/SIM failure";
				break;
			default:
				reason = "not registered";
				break;
			}
			printk("Network registration status: %d (%s)\n",
			       evt->nw_reg_status, reason);
			break;
		}

		printk("Network registration status: %s\n",
		       evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
		       "Connected - home network" : "Connected - roaming");
		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_RRC_UPDATE:
		printk("RRC mode: %s\n",
		       evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
		       "Connected" : "Idle");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		printk("LTE cell changed: Cell ID: %d, Tracking area: %d\n",
		       evt->cell.id, evt->cell.tac);
		break;
	case LTE_LC_EVT_MODEM_EVENT:
		printk("Modem event: %d\n", evt->modem_evt.type);
		break;
	default:
		break;
	}
}

/* Accuracy (m) and validity (s) reported with the initial proactive location.
 * Match these to the AT%LOCATION=2 command that registers for you manually.
 * validity 0 = infinite, appropriate for a stationary device.
 */
#define NTN_LOCATION_ACCURACY_M	100
#define NTN_LOCATION_VALIDITY_S	0

/* Provide the device position to the modem proactively, before connecting.
 * With NTN the modem needs a location to acquire a satellite (Doppler/timing
 * pre-compensation) and cannot register without it, so waiting for the modem
 * to request location first deadlocks into an endless search. This mirrors a
 * manual AT%LOCATION=2 issued before AT+CFUN=1; ntn_handler() then keeps the
 * location refreshed for any later modem requests.
 */
static void set_initial_location(void)
{
	double lat, lon;
	float alt;
	int err;

	if (parse_location(CONFIG_NTN_LOCATION, &lat, &lon, &alt) != 0) {
		printk("Invalid CONFIG_NTN_LOCATION: \"%s\"\n", CONFIG_NTN_LOCATION);
		return;
	}

	err = nrf_modem_at_printf("AT%%LOCATION=2,\"%.6f\",\"%.6f\",\"%.1f\",%d,%d",
				  lat, lon, (double)alt,
				  NTN_LOCATION_ACCURACY_M, NTN_LOCATION_VALIDITY_S);
	if (err) {
		printk("Initial location set failed, type: %d, error: %d\n",
		       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
		return;
	}
	printk("Initial NTN location set from CONFIG_NTN_LOCATION: "
	       "lat %.6f, lon %.6f, alt %.1f m\n", lat, lon, (double)alt);
}

static void modem_init(void)
{
	int err;

	err = nrf_modem_lib_init();
	if (err) {
		printk("Modem library initialization failed, error: %d\n", err);
		return;
	}

	(void)nrf_modem_at_printf("AT+CMEE=1");

	/* Lock to the NTN MSS bands. Must be done while the modem is in CFUN=0
	 * (before connect), otherwise it returns +CME ERROR 518. The band_list
	 * form is required: bands 255/256 are above the 88-bit band_mask range.
	 */
	err = nrf_modem_at_printf("AT%%XBANDLOCK=2,,\"%s\"", CONFIG_NTN_BAND_LIST);
	if (err) {
		printk("NTN band lock failed, type: %d, error: %d\n",
		       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
		return;
	}
	printk("\n\nLocked to NTN bands %s\n", CONFIG_NTN_BAND_LIST);

	/* Register for modem location requests required for NTN operation. */
	ntn_register_handler(ntn_handler);

	/* Seed the modem with our position before connecting so satellite
	 * acquisition can start (otherwise registration searches endlessly).
	 */
	set_initial_location();
}

static void modem_connect(void)
{
	int err = lte_lc_connect_async(lte_handler);

	if (err) {
		printk("Connecting to LTE network failed, error: %d\n", err);
		return;
	}
}

static void modem_power_off(void)
{
	lte_lc_power_off();
}

static int udp_pdn_setup(void)
{
	int cid = 0;
	int err;

	if (IS_ENABLED(CONFIG_UDP_ALLOC_NEW_CID)) {
		err = nrf_modem_at_scanf("AT%XNEWCID?", "%%XNEWCID: %d", &cid);
		if (err == 1) {
			printf("Created CID %d\n", cid);
		} else {
			printf("Create CID failed, error: %d\n", err);
			return -1;
		}
	}

	err = nrf_modem_at_printf("AT+CGDCONT=%d,\"IP\",\"%s\"", cid, CONFIG_UDP_APN);
	if (err == 0) {
		printk("Configured IPv4 for APN \"%s\"\n", CONFIG_UDP_APN);
	} else {
		printf("Configure PDN failed, type: %d, error: %d\n",
		       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
		return -1;
	}

	return cid;
}

static int udp_pdn_activate(int cid)
{
	char xgetpdnid[18];
	int pdn_id;
	int err;

	if (IS_ENABLED(CONFIG_UDP_ALLOC_NEW_CID)) {
		err = nrf_modem_at_printf("AT+CGACT=1,%d", cid);
		if (err == 0) {
			printk("Activated CID %d\n", cid);
		} else {
			printk("Activate CID failed, type: %d, error: %d\n",
			       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
			return -1;
		}
	}

	(void)snprintf(xgetpdnid, sizeof(xgetpdnid), "AT%%XGETPDNID=%u", cid);
	err = nrf_modem_at_scanf(xgetpdnid, "%%XGETPDNID: %d", &pdn_id);
	if (err == 1) {
		printk("Get PDN ID %d\n", pdn_id);
	} else {
		printk("Get PDN ID failed, error: %d\n", err);
		return -1;
	}

	return pdn_id;
}

static void udp_socket_close(int fd)
{
	int err;

	err = close(fd);
	if (err == 0) {
		printk("Closed socket %d\n", fd);
	} else {
		printk("Close socket failed, error: %d, errno: %d\n", err, errno);
	}
}

static int udp_socket_setup(int pdn_id)
{
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_DGRAM,
	};
	struct addrinfo *res;
	struct timeval recv_timeout = {
		.tv_sec = CONFIG_UDP_RECV_TIMEOUT_S
	};
	int fd, err;

	err = getaddrinfo(CONFIG_UDP_SERVER_ADDRESS, NULL, &hints, &res);
	if (err) {
		printk("getaddrinfo failed for \"%s\", error: %d\n",
		       CONFIG_UDP_SERVER_ADDRESS, err);
		return -1;
	}
	((struct sockaddr_in *)res->ai_addr)->sin_port = htons(CONFIG_UDP_SERVER_PORT);

	fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		printk("Create socket failed, error: %d, errno: %d\n", fd, errno);
		freeaddrinfo(res);
		return -1;
	}
	printk("Created socket %d\n", fd);

	if (IS_ENABLED(CONFIG_UDP_ALLOC_NEW_CID)) {
		err = setsockopt(fd, SOL_SOCKET, SO_BINDTOPDN, &pdn_id, sizeof(pdn_id));
		if (err == 0) {
			printk("Bound to PDN ID %d\n", pdn_id);
		} else {
			printk("Bind to PDN failed, error: %d, errno: %d\n", err, errno);
			goto error_close_socket;
		}
	}

	err = setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout));
	if (err != 0) {
		printk("Set receive timeout failed, error: %d, errno: %d\n", err, errno);
		goto error_close_socket;
	}

	/* Fix the peer so send()/recv() can be used (keeping the shell command
	 * simple). For UDP, connect() only records the default destination; it
	 * does not put any packet on the air.
	 */
	err = connect(fd, res->ai_addr, res->ai_addrlen);
	if (err != 0) {
		printk("Connect failed, error: %d, errno: %d\n", err, errno);
		goto error_close_socket;
	}
	printk("UDP socket targeting %s:%d\n", CONFIG_UDP_SERVER_ADDRESS,
	       CONFIG_UDP_SERVER_PORT);

	freeaddrinfo(res);
	return fd;

error_close_socket:
	freeaddrinfo(res);
	udp_socket_close(fd);

	return -1;
}

static void udp_send_and_recv(int fd, const void *payload, size_t payload_len)
{
	char buffer[256];
	ssize_t len;

	len = send(fd, payload, payload_len, 0);
	if (len == (ssize_t)payload_len) {
		good_sends++;
		printk("Sent %d bytes: %.*s\n", len, (int)payload_len,
		       (const char *)payload);
	} else {
		printk("Send failed, error: %d, errno: %d\n", len, errno);
		return;
	}

	/* Give the reply time to arrive before listening for it. On high-latency
	 * NTN links the response is not back immediately, so a short gap between
	 * send and receive avoids burning the recv timeout on an empty socket.
	 */
	if (CONFIG_UDP_RECV_DELAY_MS > 0) {
		k_sleep(K_MSEC(CONFIG_UDP_RECV_DELAY_MS));
	}

	len = recv(fd, buffer, sizeof(buffer) - 1, 0);
	if (len > 0) {
		good_recvs++;
		buffer[len] = '\0';
		printk("Received %d bytes: %s\n", len, buffer);
	} else if (len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
		/* recv timed out (SO_RCVTIMEO) with no reply - normal for UDP over
		 * a high-latency satellite link, not a failure.
		 */
		printk("** No reply within recv timeout: %ds\n", CONFIG_UDP_RECV_TIMEOUT_S);
	} else if (len < 0) {
		printk("Receive failed, error: %d, errno: %d\n", len, errno);
	}
}

/* Print the running send/receive tallies, prefixed with the current time as
 * UTC (converted from the date_time epoch). Shows "unknown" until time is
 * acquired.
 */
static void log_tally(void)
{
	int64_t epoch = read_unix_time();
	char when[32] = "unknown";

	if (epoch > 0) {
		time_t t = (time_t)epoch;
		struct tm tm;

		gmtime_r(&t, &tm);
		strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S UTC", &tm);
	}

	printk("Tally: time=%s cycles=%u good_sends=%u good_recvs=%u\n",
	       when, total_cycles, good_sends, good_recvs);
}

/* Read the satellite link quality once registered. %CONEVAL is rejected in
 * NTN mode (+CME ERROR 534), so use the standard +CESQ instead and decode the
 * NB-IoT fields: RSRP dBm = <rsrp> - 140, RSRQ dB = <rsrq>/2 - 19.5. A value of
 * 255 means the level is not known or not detectable.
 *
 * Returns 0 and fills *rsrp_dbm / *rsrq_db on success, 1 if the level is not
 * detectable, or -1 on query error.
 */
static int read_signal_quality(int *rsrp_dbm, double *rsrq_db)
{
	int rsrq, rsrp;
	int err;

	err = nrf_modem_at_scanf("AT+CESQ",
				 "+CESQ: %*d,%*d,%*d,%*d,%d,%d", &rsrq, &rsrp);
	if (err != 2) {
		return -1;
	}

	if (rsrp == 255) {
		return 1;
	}

	*rsrp_dbm = rsrp - 140;
	*rsrq_db = (rsrq == 255) ? 0.0 : (rsrq / 2.0) - 19.5;
	return 0;
}

/* Read the <snr> field from %XMONITOR and decode it to dB. %CONEVAL (the usual
 * SNR source) is rejected in NTN mode, but %XMONITOR carries it:
 *   <reg_status>,<full_name>,<short_name>,<plmn>,<tac>,<AcT>,<band>,<cell_id>,
 *   <phys_cell_id>,<EARFCN>,<rsrp>,<snr>,...
 * SNR is reported as an index: dB = value - 24; 127 = not known/not detectable.
 * Returns 0 and fills *snr_db on success, 1 if SNR is unavailable, -1 on error.
 */
#define XMONITOR_SNR_FIELD 11

static int read_snr(int *snr_db)
{
	char resp[160];
	const char *p;
	int field = 0;
	bool in_quote = false;
	int snr_index;

	if (nrf_modem_at_cmd(resp, sizeof(resp), "AT%%XMONITOR")) {
		return -1;
	}

	p = strchr(resp, ' '); /* skip past "%XMONITOR:" */
	if (p == NULL) {
		return -1;
	}
	p++;

	/* Advance to the start of the SNR field, ignoring commas inside the quoted
	 * operator-name/PLMN/TAC/cell-id fields.
	 */
	while (*p != '\0' && field < XMONITOR_SNR_FIELD) {
		if (*p == '"') {
			in_quote = !in_quote;
		} else if (*p == ',' && !in_quote) {
			field++;
		}
		p++;
	}
	if (field != XMONITOR_SNR_FIELD || *p == '\0' || *p == ',') {
		return 1; /* not registered yet / field empty */
	}

	snr_index = atoi(p);
	if (snr_index == 127) {
		return 1; /* not known or not detectable */
	}

	*snr_db = snr_index - 24;
	return 0;
}

static void log_signal_quality(void)
{
	int rsrp;
	double rsrq;
	int snr_db;
	int ret = read_signal_quality(&rsrp, &rsrq);

	if (ret < 0) {
		printk("Signal quality query failed\n");
	} else if (ret == 1) {
		printk("Signal quality: RSRP not detectable\n");
	} else {
		printk("Signal quality: RSRP %d dBm, RSRQ %.1f dB\n", rsrp, rsrq);
	}

	if (read_snr(&snr_db) == 0) {
		printk("Signal quality: SNR %d dB\n", snr_db);
	} else {
		printk("Signal quality: SNR not available\n");
	}
}

/* Unix epoch seconds (UTC) from the NCS date_time library. date_time obtains
 * the time from modem/network time (NITZ) first and falls back to NTP over the
 * IP connection, so it works even when the satellite network does not push time
 * directly. There is no battery RTC, so it returns 0 until a time has been
 * obtained from some source - the first send or two after boot may carry 0.
 */
static int64_t read_unix_time(void)
{
	int64_t unix_time_ms;

	if (date_time_now(&unix_time_ms) != 0) {
		return 0;
	}
	return unix_time_ms / 1000;
}

/* Render CONFIG_TEST_PAYLOADTEMPLATE into out, expanding the %-tokens documented
 * in Kconfig (%n %i %h %c %r %q %s %t %%). Unknown tokens are copied through
 * verbatim. Returns the payload length (excluding the NUL terminator), or -1 if
 * it does not fit in out_size.
 */
static int render_payload(char *out, size_t out_size, unsigned int counter)
{
	int rsrp = 0;
	double rsrq = 0.0;
	int snr_db = 0;
	bool have_signal = (read_signal_quality(&rsrp, &rsrq) == 0);
	bool have_snr = (read_snr(&snr_db) == 0);
	const char *p = CONFIG_TEST_PAYLOADTEMPLATE;
	size_t n = 0;

	while (*p != '\0') {
		int w;

		if (*p != '%') {
			if (n + 1 >= out_size) {
				return -1;
			}
			out[n++] = *p++;
			continue;
		}

		p++; /* consume '%' */
		switch (*p) {
		case 'n':
			w = snprintf(out + n, out_size - n, "%s", CONFIG_TEST_NAME);
			break;

		case 'i':
			w = snprintf(out + n, out_size - n, "%s", CONFIG_TAGO_DEVICE_TOKEN);
			break;
		case 'h':
			w = snprintf(out + n, out_size - n, "%s", CONFIG_TAGO_HASH);
			break;
        case 'c':
			w = snprintf(out + n, out_size - n, "%u", counter);
			break;
		case 'r':
			w = snprintf(out + n, out_size - n, "%d", have_signal ? rsrp : 0);
			break;
		case 'q':
			w = snprintf(out + n, out_size - n, "%.1f", have_signal ? rsrq : 0.0);
			break;
		case 't':
			w = snprintf(out + n, out_size - n, "%lld",
				     (long long)read_unix_time());
			break;
		case 's':
			w = snprintf(out + n, out_size - n, "%d", have_snr ? snr_db : 0);
			break;
		case '%':
			w = snprintf(out + n, out_size - n, "%%");
			break;
		case '\0':
			out[n] = '\0'; /* trailing '%' at end of template */
			return (int)n;
		default:
			w = snprintf(out + n, out_size - n, "%%%c", *p);
			break;
		}

		if (w < 0 || (size_t)w >= out_size - n) {
			return -1;
		}
		n += (size_t)w;
		p++;
	}

	out[n] = '\0';
	return (int)n;
}

/* Dump the full modem status line. %XMONITOR reports registration status plus
 * operator/PLMN, TAC, band, cell ID, RSRP and SNR in a single notification. The
 * field layout is variable (most fields are only present once registered), so
 * print the raw response rather than parsing each field individually.
 */
static void log_modem_status(void)
{
	char resp[160];
	int err;

	err = nrf_modem_at_cmd(resp, sizeof(resp), "AT%%XMONITOR");
	if (err) {
		printk("Modem status query failed, type: %d, error: %d\n",
		       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
		return;
	}
	printk("Modem status: %s", resp);
}

/* Shell command: send a text string over the open UDP socket and print any
 * reply. Multi-word strings must be quoted, e.g. udp send "Hello, World!".
 */
static int cmd_udp_send(const struct shell *sh, size_t argc, char **argv)
{
	if (udp_fd < 0) {
		shell_error(sh, "UDP socket not ready (not registered yet)");
		return -ENOEXEC;
	}

	udp_send_and_recv(udp_fd, argv[1], strlen(argv[1]));

	return 0;
}

/* Shell command: report the current satellite link quality and modem status. */
static int cmd_udp_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	log_signal_quality();
	log_modem_status();
	log_tally();

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(udp_subcmds,
	SHELL_CMD_ARG(send, NULL,
		      "Send a text string over UDP. Usage: udp send <text>",
		      cmd_udp_send, 2, 0),
	SHELL_CMD(status, NULL,
		  "Report satellite link quality (+CESQ) and modem status (%XMONITOR).",
		  cmd_udp_status),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(udp, &udp_subcmds, "UDP commands", NULL);

int main(void)
{
	int cid, pdn_id;

	printk("LooUQ MTC2-N9151 NTN/UDP sample started\n");
	modem_init();

	cid = udp_pdn_setup();
	if (cid == -1) {
		goto power_off;
	}

	modem_connect();
	k_sem_take(&lte_connected, K_FOREVER);

	log_signal_quality();
	log_modem_status();

	pdn_id = udp_pdn_activate(cid);
	if (pdn_id == -1) {
		goto power_off;
	}

	udp_fd = udp_socket_setup(pdn_id);
	if (udp_fd == -1) {
		goto power_off;
	}

	/* Send a freshly rendered CONFIG_TEST_TEMPLATE payload every
	 * CONFIG_TEST_INTERVAL seconds. The socket stays open between sends, so
	 * "udp send <text>" remains available for manual messages.
	 */
	printk("Sending every %d s. Manual send still available: udp send <text>\n",
	       CONFIG_TEST_INTERVAL);

	for (unsigned int counter = 0; ; counter++) {
		char payload[256];
		int len = render_payload(payload, sizeof(payload), counter);

		total_cycles++;
		if (len < 0) {
			printk("Payload does not fit buffer; check CONFIG_TEST_TEMPLATE\n");
		} else {
			udp_send_and_recv(udp_fd, payload, len);
		}
		log_tally();

		k_sleep(K_SECONDS(CONFIG_TEST_INTERVAL));
	}

power_off:
	modem_power_off();
	printk("UDP sample done\n");

	return 0;
}
