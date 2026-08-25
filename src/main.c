/*
  * Copyright (c) 2022 Nordic Semiconductor ASA
  * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
  * 
  * Copyright (c) 2023 LooUQ, Inc.
  * SPDX-License-Identifier: Apache-2.0
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
#include <zephyr/net/socket_ncs.h>

#include <date_time.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <modem/ntn.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>

K_SEM_DEFINE(lte_connected, 0, 1);

/* Given to the main thread when the GNSS receiver produces a valid fix. */
K_SEM_DEFINE(gnss_fix_sem, 0, 1);

/* Given when the date_time library reports the outcome of a time update. */
K_SEM_DEFINE(date_time_sem, 0, 1);

/* Device position supplied to the modem for NTN Doppler/timing pre-compensation.
 * In static mode it comes from CONFIG_NTN_LOCATION; in the dynamic modes from
 * the internal GNSS receiver.
 */
static double loc_lat;
static double loc_lon;
static float  loc_alt;

/* Location source, selected at build time by the GNSS mode choice in Kconfig:
 *   static        - fixed CONFIG_NTN_LOCATION, GNSS never runs
 *   dynamic-once  - one GNSS fix at boot (plus modem-requested refreshes)
 *   dynamic-every - a fresh GNSS fix before every periodic transmission
 */
#define LOCATION_IS_FIXED	IS_ENABLED(CONFIG_GNSS_MODE_STATIC)
#define LOCATION_REFIX_EVERY	IS_ENABLED(CONFIG_GNSS_MODE_DYNAMIC_EVERY)

static const char *location_mode_str(void)
{
	if (IS_ENABLED(CONFIG_GNSS_MODE_STATIC)) {
		return "static (CONFIG_NTN_LOCATION)";
	}
	if (IS_ENABLED(CONFIG_GNSS_MODE_DYNAMIC_EVERY)) {
		return "dynamic-every (GNSS fix before each transmission)";
	}
	return "dynamic-once (GNSS fix at boot)";
}

/* Set by ntn_handler() when the modem needs a fresh location and the cached fix
 * is too old to reuse. Handled by the main loop (dynamic modes only).
 */
static volatile bool refix_requested;

/* Uptime (ms) of the last GNSS fix, for the NTN_REFIX_MIN_INTERVAL_S guard. */
static int64_t last_fix_uptime;

/* Time-to-first-fix (s) of the most recent GNSS acquisition, for the shell. */
static int last_ttff_s;

/* Coarse app phase, reported by "gnss status" / "udp status". */
enum app_phase {
	PHASE_INIT,
	PHASE_GNSS_FIX,
	PHASE_ATTACHING,
	PHASE_CONNECTED,
};
static enum app_phase app_phase;

static const char *phase_str(enum app_phase p)
{
	switch (p) {
	case PHASE_GNSS_FIX:	return "acquiring GNSS fix";
	case PHASE_ATTACHING:	return "attaching to NTN";
	case PHASE_CONNECTED:	return "connected";
	default:		return "init";
	}
}

/* Latest GNSS PVT frame, filled by gnss_event_handler() in modem callback
 * context and read by the main thread once gnss_fix_sem is given.
 */
static struct nrf_modem_gnss_pvt_data_frame gnss_pvt;

/* RAI (Release Assistance Indication) configuration granted by the network,
 * from the last LTE_LC_EVT_RAI_UPDATE notification (AT%RAI=2). The network
 * advertises AS and/or CP RAI support per cell; the modem only acts on the
 * SO_RAI socket option for the mechanisms the serving cell grants.
 */
static struct lte_lc_rai_cfg rai_cfg;
static bool rai_cfg_valid;

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

/* modem_init() registers this, but it is defined later with the startup
 * helpers that use read_unix_time(). */
static void date_time_evt_handler(const struct date_time_evt *evt);

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
	int err;

	switch (evt->type) {
	case NTN_EVT_LOCATION_REQUEST:
		if (!evt->location_request.requested) {
			break;
		}

		printk("NTN location requested (accuracy %u m)\n",
		       evt->location_request.accuracy);

		/* NTN requires the modem to know the device position for
		 * Doppler/timing pre-compensation. In static mode the position never
		 * changes, so answer immediately from the cached coordinates. In the
		 * dynamic modes, reuse the cached GNSS fix if it is recent enough;
		 * otherwise ask the main loop to drop NTN and acquire a fresh fix
		 * (internal GNSS cannot run while NTN is active). dynamic-every
		 * re-acquires every cycle anyway, so its cache is rarely stale.
		 */
		if (LOCATION_IS_FIXED ||
		    (k_uptime_get() - last_fix_uptime) <
			    (int64_t)CONFIG_NTN_REFIX_MIN_INTERVAL_S * 1000) {
			err = ntn_location_set(loc_lat, loc_lon, loc_alt,
					       CONFIG_NTN_LOCATION_VALIDITY_S);
			if (err) {
				printk("ntn_location_set failed, error: %d\n", err);
			}
		} else {
			refix_requested = true;
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
#if defined(CONFIG_LTE_LC_RAI_MODULE)
	case LTE_LC_EVT_RAI_UPDATE:
		/* Tells us whether the serving cell actually grants RAI. Skylo
		 * requires the device to release the connection with RAI, so a cell
		 * reporting neither AS nor CP RAI is worth seeing in the log.
		 */
		rai_cfg = evt->rai_cfg;
		rai_cfg_valid = true;
		printk("RAI granted by cell %d (MCC %d, MNC %d): AS %s, CP %s\n",
		       rai_cfg.cell_id, rai_cfg.mcc, rai_cfg.mnc,
		       rai_cfg.as_rai ? "yes" : "no",
		       rai_cfg.cp_rai ? "yes" : "no");
		break;
#endif /* CONFIG_LTE_LC_RAI_MODULE */
	default:
		break;
	}
}

/* Accuracy (m) reported with the proactive location. Match this to the
 * AT%LOCATION=2 command that registers for you manually. The validity comes
 * from CONFIG_NTN_LOCATION_VALIDITY_S (0 = infinite, for a stationary device).
 */
#define NTN_LOCATION_ACCURACY_M	100

/* Push the device position to the modem (AT%LOCATION=2). Done before connecting
 * so satellite acquisition can start (otherwise NTN registration searches
 * endlessly); ntn_handler() keeps it refreshed for later modem requests.
 * Returns 0 on success, -1 on error.
 */
static int set_modem_location(double lat, double lon, float alt, int validity)
{
	int err = nrf_modem_at_printf(
		"AT%%LOCATION=2,\"%.6f\",\"%.6f\",\"%.1f\",%d,%d",
		lat, lon, (double)alt, NTN_LOCATION_ACCURACY_M, validity);

	if (err) {
		printk("Set location failed, type: %d, error: %d\n",
		       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
		return -1;
	}
	printk("NTN location set: lat %.6f, lon %.6f, alt %.1f m, validity %d s\n",
	       lat, lon, (double)alt, validity);
	return 0;
}

/* Called in modem library context for each GNSS event. On a valid PVT fix,
 * cache the frame and wake the acquiring thread. Keep it short.
 */
static void gnss_event_handler(int event)
{
	if (event != NRF_MODEM_GNSS_EVT_PVT) {
		return;
	}

	if (nrf_modem_gnss_read(&gnss_pvt, sizeof(gnss_pvt),
				NRF_MODEM_GNSS_DATA_PVT) == 0 &&
	    (gnss_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID)) {
		k_sem_give(&gnss_fix_sem);
	}
}

/* Put the modem in GNSS-only mode and activate the receiver. Internal GNSS and
 * NTN are mutually exclusive system modes, so the caller must have dropped the
 * NTN link first. fix_interval is passed to nrf_modem_gnss_fix_interval_set():
 * 0 = single fix, 1 = continuous navigation. Returns 0 on success, -1 on error.
 */
static int gnss_session_open(uint16_t fix_interval)
{
	int err;

	app_phase = PHASE_GNSS_FIX;

	(void)nrf_modem_at_printf("AT+CFUN=0");

	err = nrf_modem_at_printf("AT%%XSYSTEMMODE=0,0,1,0");
	if (err) {
		printk("Set GNSS system mode failed, type: %d, error: %d\n",
		       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
		return -1;
	}

	/* Activate GNSS before configuring the receiver. The nrf_modem_gnss setters
	 * return -NRF_EACCES ("GNSS is not enabled in system or functional mode")
	 * until GNSS is active in both the system mode selected above and the
	 * functional mode selected here, so this has to come first - the order used
	 * by the NCS GNSS sample. CFUN=31 leaves LTE untouched; it is already off.
	 */
	err = nrf_modem_at_printf("AT+CFUN=31");
	if (err) {
		printk("Activate GNSS failed, type: %d, error: %d\n",
		       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
		return -1;
	}

	/* Configure the receiver. Report each failure separately: the return value
	 * is a negative NRF_E* code that says why (-1 EPERM: modem library not
	 * initialized, -13 EACCES: GNSS not enabled in system/functional mode,
	 * -22 EINVAL: rejected by the GNSS stack or the receiver is still running).
	 */
	err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
	if (err) {
		printk("GNSS event handler set failed, error: %d\n", err);
		return -1;
	}

	err = nrf_modem_gnss_fix_retry_set(CONFIG_GNSS_FIX_RETRY_S);
	if (err) {
		printk("GNSS fix retry set (%d s) failed, error: %d\n",
		       CONFIG_GNSS_FIX_RETRY_S, err);
		return -1;
	}

	/* Fix retry has no effect in continuous navigation mode (fix_interval 1);
	 * it bounds the single-fix acquisitions used at boot and for refixes.
	 */
	err = nrf_modem_gnss_fix_interval_set(fix_interval);
	if (err) {
		printk("GNSS fix interval set (%u) failed, error: %d\n",
		       fix_interval, err);
		return -1;
	}

	return 0;
}

/* Stop the receiver and return the modem to CFUN=0, ready for the caller to
 * select the NTN system mode again.
 */
static void gnss_session_close(void)
{
	(void)nrf_modem_gnss_stop();
	(void)nrf_modem_at_printf("AT+CFUN=0");
}

/* Set the wall clock from the GNSS fix. The PVT frame carries UTC directly, so
 * a fix is a complete time source needing no network, no DNS and no NTP - which
 * matters here because the NTP path has to resolve and query two servers over a
 * link with a 10-20 s round trip, and Skylo does not appear to push NITZ.
 * date_time then keeps the clock running off uptime.
 *
 * Called on the first fix of each GNSS session, so the clock is re-disciplined
 * to GPS every cycle. Only announced the first time, to keep the log readable.
 */
static void set_time_from_gnss(void)
{
	bool had_time = date_time_is_valid();
	struct tm tm = {
		.tm_year = gnss_pvt.datetime.year - 1900,
		.tm_mon	 = gnss_pvt.datetime.month - 1,
		.tm_mday = gnss_pvt.datetime.day,
		.tm_hour = gnss_pvt.datetime.hour,
		.tm_min	 = gnss_pvt.datetime.minute,
		.tm_sec	 = gnss_pvt.datetime.seconds,
	};

	/* A valid fix should never carry a pre-GPS date; ignore it if it does
	 * rather than poisoning the clock the payload timestamps come from.
	 */
	if (gnss_pvt.datetime.year < 2020) {
		return;
	}

	if (date_time_set(&tm) != 0) {
		printk("Setting time from GNSS failed\n");
		return;
	}

	if (!had_time) {
		printk("Time set from GNSS: %04u-%02u-%02u %02u:%02u:%02u UTC\n",
		       gnss_pvt.datetime.year, gnss_pvt.datetime.month,
		       gnss_pvt.datetime.day, gnss_pvt.datetime.hour,
		       gnss_pvt.datetime.minute, gnss_pvt.datetime.seconds);
	}
}

/* Copy the PVT frame cached by gnss_event_handler() into loc_lat/lon/alt and
 * update the fix bookkeeping. start is the uptime the session began, used for
 * the TTFF of the first fix in that session.
 */
static void gnss_record_fix(int64_t start, bool first_of_session)
{
	loc_lat = gnss_pvt.latitude;
	loc_lon = gnss_pvt.longitude;
	loc_alt = gnss_pvt.altitude;
	last_fix_uptime = k_uptime_get();

	if (first_of_session) {
		last_ttff_s = (int)((last_fix_uptime - start) / 1000);
		set_time_from_gnss();
	}
}

/* Acquire a single position from the internal GNSS receiver. Blocks until a
 * valid fix (CONFIG_GNSS_FIX_RETRY_S = 0) or restarts the receiver on each
 * retry timeout. Used at boot and for modem-requested refixes.
 * Returns 0 on success, -1 on setup error.
 */
static int acquire_gnss_location(void)
{
	int64_t start;

	printk("Acquiring GNSS fix...\n");

	if (gnss_session_open(0) != 0) {	/* single fix */
		return -1;
	}

	start = k_uptime_get();
	for (;;) {
		if (nrf_modem_gnss_start() != 0) {
			printk("GNSS start failed\n");
			return -1;
		}

		/* With fix_retry 0 GNSS runs until a valid fix, so K_FOREVER
		 * pairs exactly; with a timeout, restart the receiver and retry.
		 */
		if (k_sem_take(&gnss_fix_sem,
			       CONFIG_GNSS_FIX_RETRY_S == 0 ? K_FOREVER :
			       K_SECONDS(CONFIG_GNSS_FIX_RETRY_S + 30)) == 0) {
			break;
		}

		printk("GNSS: no fix yet, retrying\n");
		(void)nrf_modem_gnss_stop();
	}

	gnss_record_fix(start, true);

	printk("GNSS fix: lat %.6f, lon %.6f, alt %.1f m (TTFF %d s)\n",
	       loc_lat, loc_lon, (double)loc_alt, last_ttff_s);

	gnss_session_close();

	return 0;
}

/* Run the receiver in continuous navigation mode until deadline (an uptime in
 * ms), refreshing loc_lat/lon/alt from every valid fix, so the position handed
 * to the next NTN attach is as fresh as the link schedule allows. Individual
 * fixes are logged at most every CONFIG_GNSS_TRACK_LOG_INTERVAL_S seconds to
 * keep the console readable over a long tracking window.
 *
 * Returns 0 once the window closes (whether or not it produced a fix - the
 * previous position stays valid and is reported), -1 on setup error.
 */
static int track_gnss_until(int64_t deadline)
{
	int64_t start = k_uptime_get();
	int64_t last_log = 0;
	unsigned int fixes = 0;

	printk("GNSS tracking for %lld s...\n", (deadline - start) / 1000);

	if (gnss_session_open(1) != 0) {	/* continuous navigation */
		return -1;
	}

	if (nrf_modem_gnss_start() != 0) {
		printk("GNSS start failed\n");
		gnss_session_close();
		return -1;
	}

	for (;;) {
		int64_t now = k_uptime_get();
		int64_t remaining = deadline - now;

		if (remaining <= 0) {
			break;
		}

		if (k_sem_take(&gnss_fix_sem, K_MSEC(remaining)) != 0) {
			continue;	/* window closed before the next fix */
		}

		gnss_record_fix(start, fixes == 0);
		fixes++;

		if (fixes == 1 || CONFIG_GNSS_TRACK_LOG_INTERVAL_S == 0 ||
		    (last_fix_uptime - last_log) >=
			    (int64_t)CONFIG_GNSS_TRACK_LOG_INTERVAL_S * 1000) {
			last_log = last_fix_uptime;
			printk("GNSS fix %u: lat %.6f, lon %.6f, alt %.1f m\n",
			       fixes, loc_lat, loc_lon, (double)loc_alt);
		}
	}

	gnss_session_close();

	if (fixes == 0) {
		printk("GNSS tracking window closed with no fix; "
		       "keeping previous position\n");
	} else {
		printk("GNSS tracking done: %u fixes, final lat %.6f, lon %.6f, "
		       "alt %.1f m (TTFF %d s)\n",
		       fixes, loc_lat, loc_lon, (double)loc_alt, last_ttff_s);
	}

	return 0;
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

	/* Configure the COEX0 pin that gates the external GNSS LNA/RF path before
	 * any RF activity. The modem stores this and toggles COEX0 automatically
	 * per RF frequency thereafter. Board-specific; empty string skips it.
	 */
	if (strlen(CONFIG_GNSS_COEX0_CMD) > 0) {
		err = nrf_modem_at_printf("%s", CONFIG_GNSS_COEX0_CMD);
		if (err) {
			printk("COEX0 config failed, type: %d, error: %d\n",
			       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
		} else {
			printk("COEX0 GNSS path configured: %s\n",
			       CONFIG_GNSS_COEX0_CMD);
		}
	}

	/* Register for modem location requests required for NTN operation. */
	ntn_register_handler(ntn_handler);

	/* Report time-update results for the whole session. */
	date_time_register_handler(date_time_evt_handler);
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

/* Apply a Release Assistance Indication to the socket. RAI tells the modem when
 * the exchange is finished so it can release the RRC connection immediately
 * instead of idling through the network inactivity timer - Skylo requires this
 * to tear the NTN connection down promptly. The indication is only acted on for
 * the mechanisms the serving cell grants (see LTE_LC_EVT_RAI_UPDATE), and the
 * modem also needs RAI enabled globally via CONFIG_LTE_RAI_REQ (AT%RAI).
 *
 * Compiled out entirely when CONFIG_UDP_RAI is disabled.
 */
static void udp_set_rai(int fd, int option, const char *what)
{
	if (!IS_ENABLED(CONFIG_UDP_RAI)) {
		return;
	}

	if (setsockopt(fd, SOL_SOCKET, SO_RAI, &option, sizeof(option)) != 0) {
		printk("Set RAI (%s) failed, errno: %d\n", what, errno);
	}
}

static void udp_send_and_recv(int fd, const void *payload, size_t payload_len)
{
	char buffer[256];
	int64_t sent_at;
	int64_t waited;
	ssize_t len;

	/* Declare the shape of this exchange before sending: either one reply is
	 * expected (release after it arrives) or none is (release after the send).
	 */
	udp_set_rai(fd, IS_ENABLED(CONFIG_UDP_RAI_EXPECT_REPLY) ? RAI_ONE_RESP :
								  RAI_LAST,
		    IS_ENABLED(CONFIG_UDP_RAI_EXPECT_REPLY) ? "one response" :
							      "last packet");

	len = send(fd, payload, payload_len, 0);
	sent_at = k_uptime_get();
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

	/* Time from the send completing to the reply (or to giving up), including
	 * CONFIG_UDP_RECV_DELAY_MS. This is the number to size
	 * CONFIG_UDP_RECV_TIMEOUT_S from: if replies land at 20 s the timeout has
	 * to clear that, and if none ever land no timeout will help.
	 */
	waited = k_uptime_get() - sent_at;

	if (len > 0) {
		good_recvs++;
		buffer[len] = '\0';
		printk("Received %d bytes after %lld.%01lld s: %s\n", len,
		       waited / 1000, (waited % 1000) / 100, buffer);
	} else if (len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
		/* recv timed out (SO_RCVTIMEO) with no reply - normal for UDP over
		 * a high-latency satellite link, not a failure.
		 */
		printk("** No reply within recv timeout: %ds (waited %lld.%01lld s)\n",
		       CONFIG_UDP_RECV_TIMEOUT_S, waited / 1000, (waited % 1000) / 100);
	} else if (len < 0) {
		printk("Receive failed, error: %d, errno: %d\n", len, errno);
	}

	/* Exchange over: tell the modem no more data is coming so it can release
	 * the connection now rather than waiting out the inactivity timer.
	 */
	udp_set_rai(fd, RAI_NO_DATA, "no more data");
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
		case 'l':
			/* Bare "lat,lon" so the template supplies the variable it
			 * belongs to - TagoIO TIP takes it as "location@=%l", and the
			 * same token also works appended to another variable's value.
			 * Always current: the position is set before the first send in
			 * every location mode.
			 */
			w = snprintf(out + n, out_size - n, "%.6f,%.6f",
				     loc_lat, loc_lon);
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

/* Report the RAI configuration in use: what this build asks for, and what the
 * serving cell last said it grants (%RAI notification). "not reported yet" means
 * no notification has arrived - expected before registration, or on modem
 * firmware that does not send them.
 */
static void log_rai_status(void)
{
	if (!IS_ENABLED(CONFIG_UDP_RAI)) {
		printk("RAI: disabled in this build (CONFIG_UDP_RAI=n)\n");
		return;
	}

	printk("RAI: requesting %s per send, release after exchange\n",
	       IS_ENABLED(CONFIG_UDP_RAI_EXPECT_REPLY) ? "RAI_ONE_RESP" : "RAI_LAST");

	if (IS_ENABLED(CONFIG_LTE_LC_RAI_MODULE) && rai_cfg_valid) {
		printk("RAI granted by cell %d (MCC %d, MNC %d): AS %s, CP %s\n",
		       rai_cfg.cell_id, rai_cfg.mcc, rai_cfg.mnc,
		       rai_cfg.as_rai ? "yes" : "no",
		       rai_cfg.cp_rai ? "yes" : "no");
	} else {
		printk("RAI granted by network: not reported yet\n");
	}
}

/* Print the location source, current phase, position, and (dynamic modes) the
 * age and TTFF of the last GNSS fix. Shared by "gnss status" and "udp status".
 */
static void log_location(void)
{
	printk("Location mode: %s\n", location_mode_str());
	printk("Phase: %s\n", phase_str(app_phase));
	printk("Position: lat %.6f, lon %.6f, alt %.1f m\n",
	       loc_lat, loc_lon, (double)loc_alt);

	if (LOCATION_IS_FIXED) {
		return;
	}

	if (last_fix_uptime > 0) {
		printk("Last GNSS fix: %lld s ago, TTFF %d s\n",
		       (k_uptime_get() - last_fix_uptime) / 1000, last_ttff_s);
	} else {
		printk("Last GNSS fix: none yet\n");
	}
}

/* Shell command: report the location source, phase, and last GNSS fix. */
static int cmd_gnss_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	log_location();

	return 0;
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
	log_rai_status();
	log_location();
	log_tally();

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(udp_subcmds,
	SHELL_CMD_ARG(send, NULL,
		      "Send a text string over UDP. Usage: udp send <text>",
		      cmd_udp_send, 2, 0),
	SHELL_CMD(status, NULL,
		  "Report link quality (+CESQ), modem status (%XMONITOR), and location.",
		  cmd_udp_status),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(udp, &udp_subcmds, "UDP commands", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_subcmds,
	SHELL_CMD(status, NULL,
		  "Report location source, phase, position, and last GNSS fix.",
		  cmd_gnss_status),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(gnss, &gnss_subcmds, "GNSS/location commands", NULL);

/* Switch the modem to NTN NB-IoT and bring up the UDP link with loc_lat/lon/alt
 * as the seeded position: CFUN=0 -> NTN system mode -> band lock -> set location
 * -> connect -> PDN + socket. Re-run on every (re)attach. Returns 0 on success,
 * -1 on failure.
 */
static int attach_ntn(void)
{
	int err, cid, pdn_id;

	app_phase = PHASE_ATTACHING;

	/* System mode can only change at CFUN=0. Switch to NTN NB-IoT (all other
	 * systems must be 0 for the 5th parameter to enable NTN).
	 */
	(void)nrf_modem_at_printf("AT+CFUN=0");
	err = nrf_modem_at_printf("AT%%XSYSTEMMODE=0,0,0,0,1");
	if (err) {
		printk("Set NTN system mode failed, type: %d, error: %d\n",
		       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
		return -1;
	}

	/* Lock to the NTN MSS bands. Must be done while in CFUN=0, otherwise it
	 * returns +CME ERROR 518. The band_list form is required: bands 255/256 are
	 * above the 88-bit band_mask range.
	 */
	err = nrf_modem_at_printf("AT%%XBANDLOCK=2,,\"%s\"", CONFIG_NTN_BAND_LIST);
	if (err) {
		printk("NTN band lock failed, type: %d, error: %d\n",
		       nrf_modem_at_err_type(err), nrf_modem_at_err(err));
		return -1;
	}
	printk("Locked to NTN bands %s\n", CONFIG_NTN_BAND_LIST);

	/* Seed the position before connecting so satellite acquisition can start. */
	if (set_modem_location(loc_lat, loc_lon, loc_alt,
			       CONFIG_NTN_LOCATION_VALIDITY_S) != 0) {
		return -1;
	}

	cid = udp_pdn_setup();
	if (cid == -1) {
		return -1;
	}

	k_sem_reset(&lte_connected);
	modem_connect();
	k_sem_take(&lte_connected, K_FOREVER);

	log_signal_quality();
	log_modem_status();

	pdn_id = udp_pdn_activate(cid);
	if (pdn_id == -1) {
		return -1;
	}

	udp_fd = udp_socket_setup(pdn_id);
	if (udp_fd == -1) {
		return -1;
	}

	app_phase = PHASE_CONNECTED;
	return 0;
}

/* Tear down the NTN link (close socket, power off LTE) so the GNSS system mode
 * can be selected for a fresh fix.
 */
static void detach_ntn(void)
{
	if (udp_fd >= 0) {
		udp_socket_close(udp_fd);
		udp_fd = -1;
	}
	lte_lc_power_off();
}

/* Drop the NTN link, take a fresh GNSS fix, and re-attach with it. Internal
 * GNSS and NTN cannot run at the same time, so every location refresh costs a
 * full detach/re-attach cycle. Returns 0 on success, -1 if the fix or the
 * re-attach failed. Dynamic modes only.
 */
static int refresh_location(void)
{
	detach_ntn();

	if (acquire_gnss_location() != 0 || attach_ntn() != 0) {
		return -1;
	}

	return 0;
}

/* dynamic-every duty cycle, run after each transmission: drop the NTN link and
 * track GNSS continuously until deadline (the next transmission slot), then
 * re-attach with the freshest position. deadline is an uptime in ms.
 *
 * The exchange ends with RAI_NO_DATA, so the modem has already released the
 * connection by the time this runs - no hold-off is needed before switching the
 * system mode over to GNSS.
 *
 * Returns 0 on success, -1 if GNSS setup or the re-attach failed.
 */
static int gnss_duty_cycle(int64_t deadline)
{
	/* The whole remainder of the interval belongs to GNSS. If the exchange
	 * itself consumed it, there is nothing to track: stay attached and send
	 * again rather than churning the link.
	 */
	if (deadline - k_uptime_get() <= 0) {
		printk("Exchange consumed the interval; skipping GNSS "
		       "(raise CONFIG_TEST_INTERVAL)\n");
		return 0;
	}

	detach_ntn();

	if (track_gnss_until(deadline) != 0) {
		return -1;
	}

	return attach_ntn();
}

/* Report the outcome of every date_time update, including the periodic ones the
 * library runs on its own. Without this the whole time subsystem is silent: the
 * app has no logging backend for the library's LOG_* output, so a failed sync
 * only ever showed up as "ts:=0" in the payload.
 */
static void date_time_evt_handler(const struct date_time_evt *evt)
{
	switch (evt->type) {
	case DATE_TIME_OBTAINED_MODEM:
		printk("Time obtained from modem network time (NITZ)\n");
		break;
	case DATE_TIME_OBTAINED_NTP:
		printk("Time obtained from NTP\n");
		break;
	case DATE_TIME_OBTAINED_EXT:
		printk("Time obtained from external source\n");
		break;
	case DATE_TIME_NOT_OBTAINED:
		printk("Time NOT obtained; %%t will stay 0 until a later update "
		       "succeeds\n");
		break;
	default:
		break;
	}

	k_sem_give(&date_time_sem);
}

/* Acquire wall-clock time once, while the link is up. There is no battery RTC,
 * so date_time starts with nothing; after this succeeds the library keeps the
 * clock running off uptime, and %t is good for the rest of the session.
 *
 * This has to happen here rather than being left to the library's automatic
 * update: in dynamic-every the NTN link is released right after each exchange,
 * so a background NTP fetch never gets a window to complete in.
 */
static void sync_time(void)
{
	int64_t epoch;

	if (CONFIG_TIME_SYNC_WAIT_S == 0) {
		return;
	}

	/* The dynamic modes fix GNSS before the first attach, so the clock is
	 * already set by the time we get here and there is nothing to wait for.
	 * Only static mode (GNSS never runs) needs the network for time.
	 */
	if (date_time_is_valid()) {
		printk("Time already set from GNSS; skipping network time update\n");
		return;
	}

	printk("Acquiring network time (up to %d s)...\n", CONFIG_TIME_SYNC_WAIT_S);

	k_sem_reset(&date_time_sem);
	if (date_time_update_async(NULL) != 0) {
		printk("Time update request failed\n");
		return;
	}

	if (k_sem_take(&date_time_sem, K_SECONDS(CONFIG_TIME_SYNC_WAIT_S)) != 0) {
		printk("Time update did not complete within %d s\n",
		       CONFIG_TIME_SYNC_WAIT_S);
		return;
	}

	epoch = read_unix_time();
	if (epoch > 0) {
		time_t t = (time_t)epoch;
		struct tm tm;
		char when[32];

		gmtime_r(&t, &tm);
		strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S UTC", &tm);
		printk("Time set: %s (epoch %lld)\n", when, (long long)epoch);
	}
}

int main(void)
{
	printk("LooUQ MTC2-N9151 NTN/UDP sample started\n");
	modem_init();

	printk("Location mode: %s\n", location_mode_str());

	/* Static mode needs a parsable CONFIG_NTN_LOCATION; the dynamic modes seed
	 * the position from GNSS before the first attach (both of them: NTN cannot
	 * register without a position, so dynamic-every also fixes at boot).
	 */
	if (LOCATION_IS_FIXED) {
		if (parse_location(CONFIG_NTN_LOCATION, &loc_lat, &loc_lon,
				   &loc_alt) != 0) {
			printk("CONFIG_NTN_LOCATION (\"%s\") is not a valid "
			       "\"lat,lon,alt\" string; aborting\n", CONFIG_NTN_LOCATION);
			goto power_off;
		}
		printk("Using fixed location from CONFIG_NTN_LOCATION: "
		       "lat %.6f, lon %.6f, alt %.1f m\n",
		       loc_lat, loc_lon, (double)loc_alt);
	} else if (acquire_gnss_location() != 0) {
		printk("GNSS acquisition failed; aborting\n");
		goto power_off;
	}

	if (attach_ntn() != 0) {
		goto power_off;
	}

	/* Get the clock while the link is up, so the first payload already carries
	 * a real %t timestamp.
	 */
	sync_time();

	/* Send a freshly rendered CONFIG_TEST_TEMPLATE payload every
	 * CONFIG_TEST_INTERVAL seconds. The socket stays open between sends, so
	 * "udp send <text>" remains available for manual messages.
	 */
	printk("Sending every %d s. Manual send still available: udp send <text>\n",
	       CONFIG_TEST_INTERVAL);
	if (LOCATION_REFIX_EVERY) {
		printk("Each cycle: send, release with RAI, then GNSS tracking until "
		       "the next send\n");
	}

	for (unsigned int counter = 0; ; counter++) {
		char payload[256];
		int64_t deadline = k_uptime_get() +
				   (int64_t)CONFIG_TEST_INTERVAL * 1000;
		int len = render_payload(payload, sizeof(payload), counter);

		total_cycles++;
		if (len < 0) {
			printk("Payload does not fit buffer; check CONFIG_TEST_TEMPLATE\n");
		} else {
			udp_send_and_recv(udp_fd, payload, len);
		}
		log_tally();

		/* dynamic-every: spend the rest of the interval on continuous GNSS
		 * tracking, so the next send goes out on a just-acquired position.
		 * The boot fix covers the first one.
		 */
		if (LOCATION_REFIX_EVERY) {
			if (gnss_duty_cycle(deadline) != 0) {
				printk("GNSS duty cycle / re-attach failed; aborting\n");
				goto power_off;
			}
			continue;	/* the duty cycle consumed the interval */
		}

		/* The modem asked for a fresh location and the cached fix is stale.
		 * Internal GNSS cannot run while NTN is active, so drop the link,
		 * re-acquire, and re-attach. (dynamic-once only; dynamic-every
		 * already refreshes every cycle.)
		 */
		if (!LOCATION_IS_FIXED && refix_requested) {
			printk("Refreshing GNSS location on modem request\n");
			refix_requested = false;
			if (refresh_location() != 0) {
				printk("Location refresh / re-attach failed; aborting\n");
				goto power_off;
			}
		}

		k_sleep(K_SECONDS(CONFIG_TEST_INTERVAL));
	}

power_off:
	modem_power_off();
	printk("UDP sample done\n");

	return 0;
}
