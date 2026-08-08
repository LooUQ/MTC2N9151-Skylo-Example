# Cellular ↔ Satellite Failover — Design Plan

**Status:** proposed, not implemented
**Target:** `mtc2n9151/nrf9151/ns` (LooUQ MTC2-N9151)
**Modem firmware:** `mfw_nrf9151-ntn_1.0.0`
**SDK:** nRF Connect SDK v3.3.1

## Goal

Detect loss of terrestrial cellular coverage and fail over to Skylo NTN, then return to
cellular when it recovers. Both transitions are gated by configurable delays.

This inverts the current application, which is satellite-only: today `main()` attaches to NTN
once and aborts on any failure (`src/main.c`, `main()` / `attach_ntn()`). Under this plan
**cellular is primary and satellite is the fallback**, and no single attach failure is fatal.

Throughout, **TN** = terrestrial cellular (LTE-M or NB-IoT), **NTN** = satellite NB-IoT.

---

## 1. Constraints that shape the design

All verified against the NTN AT command reference, the `mfw_nrf9151-ntn_1.0.0` release notes,
and *NTN operation v1.4* (in `C:\ncs\1.0.0-documentation-package.zip`).

| Constraint | Consequence |
|---|---|
| `%XSYSTEMMODE`: NTN NB-IoT cannot be combined with LTE-M/NB-IoT — `<NTN_NB_IoT_support>=1` requires all other parameters `0` | Only one access is live at a time. There is no "both radios listening". Every switch is a modem system-mode change. |
| `+CFUN=45` = flight mode **preserving LTE registration context** (`LTE_LC_FUNC_MODE_OFFLINE_KEEP_REG`). Allowed only when two cellular profiles are configured. | The fast-switch primitive: `45 → %XSYSTEMMODE → 1`, with no DETACH/TAU/re-ATTACH. |
| `%CELLULARPRFL` present in 1.0.0 (release notes list fixes for it). `CID = cid(0..9) + 10 × CPID` | NTN PDN on CID 0, TN PDN on CID 10. Two `+CGDCONT` entries — **both carrying the same APN**. |
| `%XBANDLOCK` is global and AND-ed. NTN bands 23/255/256 are not selectable while in TN system mode. | Set the **union** of TN+NTN bands **once at boot while in NTN mode**. The current per-attach `%XBANDLOCK` in `attach_ntn()` will fail outright in TN mode. |
| Context preservation is automatic only for Skylo SIMs; otherwise `AT%SKYLO=0,1` | Needed explicitly for the Monogoto SIM. |
| GNSS coexists with TN (`%XSYSTEMMODE=1,0,1,0`) but not with NTN | Position refresh is free while on cellular. On NTN it needs the mode dance, but can use `CFUN=45` rather than `CFUN=0` so a refix no longer costs a full re-attach. |
| `%CONEVAL` / `%NCELLMEAS` unsupported on NTN, supported on TN | The existing `+CESQ` / `%XMONITOR` workaround stays. `lte_lc_conn_eval_params_get()` is additionally available for TN health. |

### The single-SIM caveat

*NTN operation v1.4* §7 warns explicitly: with one subscription across both accesses, the network
may reject the UE-initiated service request after a switch, forcing a re-ATTACH — or accept it
while downlink traffic silently stops. "Currently the network behaviour is undeterministic from
the UE point of view."

**Re-attach is therefore a normal outcome, not a bug.** The state machine must recover from it
rather than treating it as failure, and the rate at which it happens is itself a useful field-test
metric.

---

## 2. One APN, two contexts

There is exactly one APN: **`go.mono`** (Monogoto), which roams onto Skylo for satellite access.
Both `go.mono` and `skylo.ip` have been tested successfully; `prj.conf` currently carries
`skylo.ip` only because that is what the present build happens to hold.

Profile-based switching still needs two `+CGDCONT` contexts, because the context ID is derived
from the profile index — but both carry the same APN string. Nordic's own §9.3 example does
exactly this:

```
at+cgdcont=0,"ip","skylo.ip"
at+cgdcont=10,"ip","skylo.ip"
```

Two contexts, one APN. `CONFIG_UDP_APN` is renamed `CONFIG_LINK_APN` and remains the single
source of truth, written to every context.

---

## 3. Recovery cannot be passive

While the modem is in NTN system mode it is **completely blind to terrestrial coverage** — no
RSSI, no cell list, nothing. "Cellular recovered" has to be actively probed by temporarily
leaving satellite.

So a single "return delay" is not implementable. It decomposes into:

- **`LINK_TN_PROBE_INTERVAL_S`** — minimum satellite dwell before spending a probe.
- **`LINK_TN_PROBE_TIMEOUT_S`** — how long a probe may search before abandoning.
- **`LINK_TN_RECOVERY_DELAY_S`** — how long TN must stay healthy after a successful probe
  before committing.
- **`LINK_TN_PROBE_BACKOFF_MAX_S`** — failed probes double the interval up to this ceiling,
  so a device parked out of coverage does not probe itself flat.

The loss direction genuinely is one knob: **`LINK_TN_LOSS_DELAY_S`**.

---

## 4. Architecture

Add `src/link.c` / `src/link.h`. `main.c` keeps payload rendering, telemetry and shell commands;
it stops owning the modem.

```
                        loss debounce expires
      TN_UP ──────────────────────────────────────► SWITCHING_TO_NTN ──► NTN_UP
        ▲ ▲                                                                │
        │ └── health recovers, timer reset                                 │ probe interval
        │                                                                  ▼
        │            recovery delay elapsed, still healthy             TN_PROBE
        └──────────────────────────────────────────────────────────────────┤
                                            probe timeout / probe failed  │
                                                                          ▼
                                                          NTN_UP (interval × backoff)

      both unhealthy (real or overridden) ──► NO_SERVICE ──► alternating retry w/ backoff
```

States: `INIT`, `TN_UP`, `TN_DEGRADED`, `SWITCHING_TO_NTN`, `NTN_UP`, `TN_PROBE`,
`SWITCHING_TO_TN`, `NO_SERVICE`.

Run the manager on its own thread, fed by a `k_msgq` from `lte_handler()` and from the send/recv
path. Switching must not run on the main thread — an attach can block for minutes, and the
payload cadence should not be hostage to it.

### `NO_SERVICE`

Reached when both accesses are unhealthy.

- **Alternate retry attempts between accesses.** Retrying only the primary would mean never
  noticing the fallback recovered.
- **Backoff:** `LINK_NO_SERVICE_RETRY_S` doubling to `LINK_NO_SERVICE_RETRY_MAX_S`.
- **Park the modem between attempts** (`CFUN=4`, or `CFUN=45` under mechanism B). Left in normal
  mode a device in a coverage hole burns continuous-search power indefinitely.
- **Payloads are dropped and counted separately** in `log_tally()` as outage-drops, distinct from
  send failures.

---

## 5. Loss detection

Cellular is unhealthy if **any** of these holds. Any one true starts/holds the loss debounce;
all false resets it.

1. **Registration** — `LTE_LC_EVT_NW_REG_STATUS` reports anything other than
   `REGISTERED_HOME` / `REGISTERED_ROAMING`. Note the Monogoto subscription roams, so
   **roaming is normal** — do not gate anything on "home network".
   Weight `LTE_LC_NW_REG_NO_SUITABLE_CELL` (91) heavily; the NCS header documents it as
   *"may be used as a trigger for changing the configured system mode."*
2. **Search exhaustion** — `LTE_LC_MODEM_EVT_SEARCH_DONE` while unregistered means all
   frequencies were scanned with nothing suitable found. Strongest single signal.
   Requires `CONFIG_LTE_LC_MODEM_EVENTS_MODULE=y` (drives `AT%MDMEV=2`).
3. **Application failure** — `LINK_FAIL_SENDS` consecutive failed send / no-reply cycles.
   Catches "registered but the bearer is dead", which registration cannot see.
4. *(optional, off by default)* **RSRP floor** — `LINK_TN_RSRP_MIN_DBM`. Left disabled: NB-IoT
   works at signal levels that look alarming, so a threshold generates false failovers more
   readily than real ones.

Registration state and `SEARCH_DONE` have no tunables — they are categorical, not
threshold-based.

Which access is actually live should be read from the `<AcT>` field of `%XMONITOR`
(7 = LTE-M, 9 = NB-IoT, 14 = NTN NB-IoT) rather than inferred from what the manager last
commanded. Both accesses are the same operator, so registration state cannot distinguish them,
and reading `AcT` also catches a switch that silently did not take.

---

## 6. Switch sequences

### Boot, once (in NTN system mode, modem off)

```
AT+CFUN=0
AT%XSYSTEMMODE=0,0,0,0,1
AT%XBANDLOCK=2,,"<TN bands>,23,255,256"   ; union — NTN mode is the only place both bind
AT%SKYLO=0,1                              ; non-Skylo SIM: enable context preservation
AT%CELLULARPRFL=2,0,4,0                   ; CPID 0 = NTN
AT%CELLULARPRFL=2,1,1,0                   ; CPID 1 = TN  (1 = LTE-M, 2 = NB-IoT)
AT%CELLULARPRFL=1                         ; profile-change notifications
AT+CGDCONT=0,"IP","go.mono"
AT+CGDCONT=10,"IP","go.mono"
```

Under mechanism A (below) the `%SKYLO` and `%CELLULARPRFL` lines and the second `+CGDCONT` are
omitted.

### TN → NTN

1. Close socket
2. `CFUN=45` (or `CFUN=0` under mechanism A)
3. `%XSYSTEMMODE=0,0,0,0,1`
4. Refresh `%LOCATION=2,…` — re-acquire via GNSS first if the fix is older than
   `NTN_LOCATION_VALIDITY_S`
5. `CFUN=1`
6. Await registration, bounded by `LINK_ATTACH_TIMEOUT_NTN_S`
7. `%XGETPDNID=0` → `SO_BINDTOPDN` → socket

### NTN → TN

1. Close socket
2. `CFUN=45` (or `CFUN=0`)
3. `%XSYSTEMMODE=1,0,1,0,0` — LTE-M + GNSS, or NB-IoT per `LINK_TN_MODE`
4. `CFUN=1` (or `CFUN=2` for a probe)
5. Await registration, bounded by `LINK_ATTACH_TIMEOUT_TN_S`
6. `%XGETPDNID=10` → `SO_BINDTOPDN` → socket

Neither path touches `%XBANDLOCK`.

### Switching mechanisms

Because there is only one APN, a single context is sufficient — which makes the whole
`%CELLULARPRFL` layer *optional*, existing only to buy `CFUN=45` context preservation.
Make it a Kconfig choice and build the simple one first.

- **Mechanism A — `LINK_SWITCH_MECHANISM=SIMPLE`.** No profiles, single CID 0, `CFUN=0`
  switches. Nothing to misconfigure. Costs a full ATTACH each way and writes NVM on every
  `CFUN=0`.
- **Mechanism B — `LINK_SWITCH_MECHANISM=KEEP_CONTEXT`.** Profiles, CID 0 / CID 10, `CFUN=45`.
  Faster and far less signalling, but inherits the §1 single-SIM caveat.

### Cheap probe

With `LINK_PROBE_RX_ONLY=y`, probe using `CFUN=2` (receive-only: search and measure, no
transmit). Registration is reported via the `LTE_LC_NW_REG_RX_ONLY_*` states (50–55); promote to
`CFUN=1` only on `RX_ONLY_REGISTERED_HOME`/`ROAMING`. Saves considerable energy on failed probes.

The AT reference notes `CFUN=1/21 → 2` is unsupported; the path here is `45 → 2`, which should be
legal. Verify on hardware — the full-attach probe is the guaranteed fallback.

### Re-attach escape hatch

If a `CFUN=45` switch lands in `REGISTRATION_DENIED`, or sends fail immediately after a switch,
the network refused the preserved context. Fall back to `CFUN=0` → `CFUN=1` for a clean attach,
and count these separately.

### Keep the GNSS fix warm

`NTN_LOCATION_VALIDITY_S = 3600` means that after an hour on cellular the modem's position has
expired, and the next failover must wait out a GNSS TTFF before it can even begin. GNSS coexists
with TN mode, so **refresh the fix opportunistically while on cellular** — failover then never
pays for time-to-first-fix.

---

## 7. GPIO test overrides

Two pins force a link to *evaluate* as bad, so the real timing logic runs unmodified. This is
what makes phases 3–4 testable on a bench instead of by driving around hunting coverage holes.

Both are **one-directional: they force bad, never good.** Releasing a pin only removes the
synthetic fault; a genuinely dead link stays dead and normal detection still rules.

`boards/mtc2n9151_nrf9151_ns.overlay`:

```dts
/ {
	link_overrides {
		compatible = "gpio-keys";
		tn_bad_override: tn_bad {
			gpios = <&gpio0 13 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
			label = "Force cellular bad";
		};
		ntn_bad_override: ntn_bad {
			gpios = <&gpio0 14 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
			label = "Force NTN bad";
		};
	};
};
```

P0.13 and P0.14 are unassigned in the board dtsi (P0.00–0.05, 0.07–0.10, 0.30, 0.31 are taken by
I2C2/UART0/SPI1/I2C3; P0.26 is reserved for the commented-out PWM). `&gpiote` is already enabled.
**Confirm both pins are broken out and free on your carrier before committing.** The modem's
COEX0 (`%XCOEX0`) is a dedicated modem pin, not `gpio0`, so it does not conflict.

Active-low with internal pull-up: open pin = normal, short to GND = forced bad. Fails safe if a
wire falls off, and one 3-pin jumper block (P0.13 / P0.14 / GND) reaches every test case. Each pin
is guarded independently with `DT_NODE_EXISTS`, so either or neither may be populated.

### Override matrix

| P0.13 (TN bad) | P0.14 (NTN bad) | Behaviour |
|---|---|---|
| — | — | Normal automatic operation. |
| asserted | — | TN evaluates bad → loss debounce → switch to NTN. Probes are failed on evaluation while asserted, so a probe cannot find real coverage and return early. |
| — | asserted | NTN ineligible. **On NTN:** immediate TN probe — no reason to sit on a link declared dead. **On TN:** stay put; a subsequent TN loss does *not* fall back, it goes to `NO_SERVICE`. |
| asserted | asserted | Both ineligible → `NO_SERVICE`. Asserted at boot, reached without ever activating the radio. |

Attach attempts to an overridden access **fail fast on evaluation rather than being attempted** —
consistent with "this access evaluates as bad", and it avoids burning minutes on an NTN attach
intended to fail.

### Implementation notes

**Poll the pins on the existing health tick — do not use an interrupt.** One tick of latency is
meaningless against a 120 s loss delay, and polling eliminates contact bounce and ISR-context
concerns for free. GPIO from the non-secure image is fine on this `ns` target.

Log every transition loudly (`Cellular-bad override ASSERTED` / `RELEASED`) so a capture is
unambiguous about which switches were synthetic.

### Shell parity

`link bad tn on|off|auto`, `link bad ntn on|off|auto`, `link bad none`. Effective override per
access is `pin asserted OR shell override`, so every case is reachable over serial with nothing
wired. `link status` shows both override states and which source drives each.

---

## 8. Configuration

Full explanation of every symbol, a worked timeline, the interaction traps and tuning presets:
**[failover-config.md](failover-config.md)**. A browser-readable copy of the same reference is
published at <https://claude.ai/code/artifact/80c3c8b1-aff0-4ffa-8a81-82cba3d788b3>; the file in
this repo is the authoritative version.

New symbols, declared in `Kconfig` and set in `prj.conf`:

```
LINK_APN                        "go.mono"
LINK_PRIMARY                    TN | NTN
LINK_TN_MODE                    LTEM | NBIOT
LINK_TN_BAND_LIST               "2,4,12,13,66"
LINK_SWITCH_MECHANISM           SIMPLE | KEEP_CONTEXT

LINK_TN_LOSS_DELAY_S            120
LINK_TN_RECOVERY_DELAY_S        300

LINK_TN_PROBE_INTERVAL_S        900
LINK_TN_PROBE_TIMEOUT_S         180     # see traps — likely too low for this build
LINK_TN_PROBE_BACKOFF_MAX_S     3600
LINK_PROBE_RX_ONLY              y

LINK_FAIL_SENDS                 3
LINK_TN_RSRP_MIN_DBM            0       # 0 = disabled
LINK_ATTACH_TIMEOUT_TN_S        300
LINK_ATTACH_TIMEOUT_NTN_S       900

LINK_OVERRIDE_GPIO              n
LINK_OVERRIDE_PROBE_ON_RELEASE  y
LINK_OVERRIDE_SKIP_ATTACH       y
LINK_NO_SERVICE_RETRY_S         300     # see traps — use 600
LINK_NO_SERVICE_RETRY_MAX_S     3600
```

Defaults are proposals, not measured optima. Expect the first field session to move at least
`LINK_TN_PROBE_TIMEOUT_S` and `LINK_ATTACH_TIMEOUT_NTN_S`.

### `prj.conf` changes

- **Remove** `CONFIG_LTE_NETWORK_MODE_NTN_NBIOT=y` — `lte_lc` applies that system mode at init
  and would fight the link manager.
- **Add** `CONFIG_LTE_LC_MODEM_EVENTS_MODULE=y` for `SEARCH_DONE`.
- **Keep** `CONFIG_LTE_PLMN_SELECTION_OPTIMIZATION=n`. It is required for Skylo registration, and
  since it is applied once globally there is no way to enable it for TN only. Accept the slower
  terrestrial search — and budget the probe timeout accordingly.

---

## 9. Changes to existing code

| Location | Change |
|---|---|
| `attach_ntn()` | → `link_attach(enum link_rat)`, parameterised on system mode / CID / timeout. Remove the per-attach `%XBANDLOCK`. Replace the unbounded `k_sem_take(&lte_connected, K_FOREVER)` with a bounded wait — an unbounded wait is fatal to a fallback design. |
| `detach_ntn()` | → `link_detach(bool keep_context)`, using `CFUN=45` vs `lte_lc_power_off()`. |
| `acquire_gnss_location()` | Use `CFUN=45 / 30 / 31` instead of `CFUN=0` so a refix preserves the NTN attach. Skip the mode dance entirely on TN, where GNSS already coexists. |
| `lte_handler()` | Post events to the manager queue. Handle `LTE_LC_EVT_MODEM_EVENT` properly (currently just prints a number) and the `RX_ONLY_*` registration states. |
| `udp_send_and_recv()` | Return a status so the consecutive-failure counter can drive detection. |
| `main()` loop | Hand payloads to the manager, which may hold or drop them mid-switch. |
| `render_payload()` | Add `%a` (access tech, from `%XMONITOR` `AcT`) and `%o` (override state: `-`, `T`, `N`, `TN`). Without `%o`, synthetic bench switches are indistinguishable from real coverage findings in TagoIO six weeks later. |
| `log_tally()` | Add switch counts, per-access send/recv tallies, time on each access, forced-re-attach count, outage-drops. For a field-test tool this is arguably the most valuable output of the feature. |
| Shell | `link status`, `link force tn|ntn|auto`, `link bad …`. |

---

## 10. Phasing

1. **Refactor only** — link manager owning a single access, bounded attach waits, APN renamed and
   set to `go.mono`. No behavioural change. Proves the state machine against the network that
   already works.
2. **Mechanism A + `link force tn|ntn|auto`** — manual switching, validated by hand in the field.
3. **GPIO overrides + `NO_SERVICE`, then automatic loss detection** with `LINK_TN_LOSS_DELAY_S`.
   The overrides land first because they are what make the rest testable.
4. **Probing and recovery** — `LINK_TN_PROBE_*`, `LINK_TN_RECOVERY_DELAY_S`, backoff, telemetry.
5. **Mechanism B** — only if phase 2–4 field data shows the re-attach cost actually hurts.

Note that `link force` (phase 2) and the override pins (phase 3) are complementary, not
duplicates: `link force` tests the switching *mechanism*, the overrides test the *policy and
timing*.

---

## 11. Open items to confirm on hardware

1. **Does the Monogoto subscription permit terrestrial registration in the test areas, and on
   which bands?** This sets `LINK_TN_BAND_LIST`, and it is the one assumption that could
   invalidate the whole approach. Worth settling before writing code.
2. **`CFUN=45` with a single physical SIM and two profiles on mfw 1.0.0.** *NTN operation v1.4*
   §7 documents exactly this, but the NCS header describes `CFUN=45` in terms of dual-UICC. Only
   gates phase 5; if it fails, everything degrades to `CFUN=0` switches.
3. **`CFUN=45 → CFUN=2`** — determines whether the cheap probe is available.
4. **How often the network rejects the preserved context after a switch** — decides whether
   mechanism B is worth keeping at all.
5. **NVM wear.** `CFUN=0` writes NVM and `%XSYSTEMMODE` persists periodically. `CFUN=45` avoids
   most of it, but the minimum dwell timers are also a wear guard, not merely an anti-thrash
   measure. `LINK_TN_PROBE_INTERVAL_S` must not be tuned down to seconds.
6. **P0.13 / P0.14 are broken out and unused** on the LooUQ Breakout / UXplor carrier.
