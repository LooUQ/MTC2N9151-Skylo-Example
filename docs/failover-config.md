# Failover Configuration Reference

Every Kconfig setting governing cellular ↔ satellite failover, and how they interact.

**Status:** proposed, not implemented — see [failover-plan.md](failover-plan.md)
**Target:** `mtc2n9151/nrf9151/ns` · **Modem:** `mfw_nrf9151-ntn_1.0.0` · **SDK:** NCS v3.3.1

New symbols are declared in `Kconfig` and given values in `prj.conf`, alongside the settings
already there. Throughout, **TN** = terrestrial cellular, **NTN** = satellite. The distinction
matters because the modem can only run one of them at a time, which is the single constraint that
shapes every setting below.

Twenty-two settings govern when this device abandons cellular for satellite and when it comes
back. Most are obvious in isolation. The ones that cause trouble are the pairs whose defaults
quietly contradict each other — see [Interaction traps](#interaction-traps).

---

## 1. Topology — which network, and how to switch

Set once for a given deployment; these do not change at runtime.

| Symbol | Default | Radio | What it controls |
|---|---|---|---|
| `LINK_APN` | `"go.mono"` | both | The single APN, written to every PDN context. Renamed from `UDP_APN`. One APN covers both accesses — the Monogoto subscription roams onto Skylo, so there is never a second one. |
| `LINK_PRIMARY` | `TN` | both | Which access is preferred. Choice of `TN` / `NTN`. Setting `NTN` restores today's satellite-only behaviour. |
| `LINK_TN_MODE` | `LTEM` | TN | Terrestrial access technology. Choice of `LTEM` / `NBIOT`. Feeds the `%XSYSTEMMODE` written on each switch. |
| `LINK_TN_BAND_LIST` | `"2,4,12,13,66"` | TN | Terrestrial bands, merged with the existing `NTN_BAND_LIST` into one union `%XBANDLOCK` issued at boot. |
| `LINK_SWITCH_MECHANISM` | `SIMPLE` | both | `SIMPLE` uses `CFUN=0` switches on a single context. `KEEP_CONTEXT` uses cellular profiles and `CFUN=45` to preserve registration across a switch. This choice changes the cost of everything in §3. |

---

## 2. The two headline delays

The settings the feature exists to expose: how patient the device is before giving up on
cellular, and how sure it wants to be before trusting it again.

| Symbol | Default | What it controls |
|---|---|---|
| `LINK_TN_LOSS_DELAY_S` | 120 | Cellular must evaluate as unhealthy continuously for this long before switching to satellite. A single healthy evaluation resets the timer to zero, so a flapping link does not accumulate credit toward a switch. |
| `LINK_TN_RECOVERY_DELAY_S` | 300 | After a probe finds cellular again, it must stay healthy this long before the device commits. Degrading during the hold-down returns to satellite immediately — there is no second loss delay, because the link was never committed to. |

---

## 3. Recovery probing

Recovery cannot be a passive observation. While the modem is in satellite mode it is *completely
blind* to terrestrial coverage — no RSSI, no cell list, nothing. So "cellular came back" has to be
discovered by temporarily leaving satellite and looking, which is why one conceptual knob becomes
four real ones.

| Symbol | Default | What it controls |
|---|---|---|
| `LINK_TN_PROBE_INTERVAL_S` | 900 | Minimum satellite dwell before spending a probe, measured from entering the satellite-up state. The closest thing to a plain "return delay". |
| `LINK_TN_PROBE_TIMEOUT_S` | 180 | How long a probe may search before abandoning and returning to satellite. See the search-timing trap — this default is probably too low for this build. |
| `LINK_TN_PROBE_BACKOFF_MAX_S` | 3600 | Each failed probe doubles the interval, up to this ceiling. Resets on a successful commit. Keeps a device parked in a coverage hole from probing itself flat. |
| `LINK_PROBE_RX_ONLY` | `y` | Probe with `CFUN=2` — search and measure without transmitting — instead of a full attach, promoting to a real attach only once a suitable cell appears. Materially cheaper on failed probes. |

---

## 4. Health detection

Four inputs decide whether cellular is healthy. Any one reading bad marks the link bad and starts
the loss delay; all must read good to reset it.

| Symbol | Default | What it controls |
|---|---|---|
| `LINK_FAIL_SENDS` | 3 | Consecutive failed send / no-reply cycles that mark the link unhealthy. Catches the case registration cannot see: registered normally, but the bearer is dead. |
| `LINK_TN_RSRP_MIN_DBM` | 0 *(off)* | RSRP floor below which cellular counts as bad. Disabled deliberately — NB-IoT works reliably at signal levels that look alarming on a meter, so a threshold here generates false failovers more readily than real ones. |
| `LINK_ATTACH_TIMEOUT_TN_S` | 300 | Abandon a terrestrial attach after this long. |
| `LINK_ATTACH_TIMEOUT_NTN_S` | 900 | Abandon a satellite attach after this long. See the attach-patience trap. |

### Two inputs have no knobs

Registration state and the modem's `SEARCH_DONE` event are always-on, because they are categorical
rather than threshold-based. `SEARCH_DONE` while unregistered means every frequency was scanned
and nothing suitable was found — the strongest single signal available, and worth more than any
amount of RSRP averaging. The `NO_SUITABLE_CELL` registration state is documented in the SDK
headers as a trigger for changing system mode, which is precisely what we use it for.

---

## 5. Test overrides and no-service

The override pins let a bench exercise the full failover and recovery path without driving
anywhere. They force a radio to *evaluate* as bad, so the real timing logic runs unmodified — and
they are one-directional: they can force bad, never good, so a genuine outage is never masked by a
jumper.

| Symbol | Default | What it controls |
|---|---|---|
| `LINK_OVERRIDE_GPIO` | `n` | Enables the override inputs: `P0.13` forces cellular bad, `P0.14` forces satellite bad. Both active-low with internal pull-ups, so an open pin is normal and a short to ground is the fault. Active level and pull live in the devicetree overlay, not here. |
| `LINK_OVERRIDE_PROBE_ON_RELEASE` | `y` | Releasing a pin triggers an immediate probe rather than waiting out the probe interval. Without this, testing the return path against a 900 s interval is tedious enough that people stop doing it. |
| `LINK_OVERRIDE_SKIP_ATTACH` | `y` | An overridden radio is not attached at boot at all, rather than attached and then abandoned. Makes satellite-first cold start a one-jumper test. |
| `LINK_NO_SERVICE_RETRY_S` | 300 | Retry cadence when both radios are unhealthy. Attempts alternate between them — retrying only the primary would mean never noticing the fallback recovered. |
| `LINK_NO_SERVICE_RETRY_MAX_S` | 3600 | Backoff ceiling for that retry loop. Between attempts the modem is parked rather than left searching, which is the difference between a weekend and a month of battery in a coverage hole. |

---

## 6. Inherited settings that now interact

These already exist in `prj.conf`. Failover gives them new significance, so they belong in any
tuning pass.

| Symbol | Current | Why it matters now |
|---|---|---|
| `TEST_INTERVAL` | 120 | Sets the send cadence, and therefore the clock rate of the `LINK_FAIL_SENDS` detector. Three failures at 120 s is six minutes — that detector can never be fast. |
| `NTN_LOCATION_VALIDITY_S` | 3600 | How long the modem trusts the position it was given. Sit on cellular longer than this and the position has expired, so the next failover has to wait out a GNSS fix before it can even start. |
| `NTN_REFIX_MIN_INTERVAL_S` | 600 | Floor on full GNSS re-acquisitions. Bounds how often a position refresh can interrupt the link. |
| `LTE_PLMN_SELECTION_OPTIMIZATION` | `n` | Must stay off for Skylo to register at all, but it slows terrestrial searches too — and terrestrial search speed is exactly what the probe timeout budgets for. |

---

## 7. Worked timeline

Cellular coverage dies at t=0, with every default as listed above.

| Time (s) | Radio | Event |
|---:|---|---|
| 0 | TN | **Coverage lost.** Registration drops to searching; health evaluates bad. Loss delay starts. |
| 0 – 120 | TN | Sends still attempted, still failing. The failure counter is climbing but will not reach 3 before the timer expires. `LINK_TN_LOSS_DELAY_S = 120` |
| 120 | switching | **Failover.** Close socket, drop the modem, write the satellite system mode, seed the position, reactivate. `CFUN → %XSYSTEMMODE → %LOCATION → CFUN=1` |
| 300 | NTN | **Satellite registered** and sending. Took 180 s here; the ceiling is 900. Bounded by `LINK_ATTACH_TIMEOUT_NTN_S` |
| 1200 | switching | **Probe due** — 900 s of satellite dwell elapsed. The device leaves satellite to look for cellular. |
| ↳ | | *nothing found* → return to satellite within 180 s, interval doubles to 1800 |
| ↳ | | *cell found* → registers at t=1260, hold-down begins |
| 1260 – 1560 | TN | On cellular and sending, but not committed. Any degradation here goes straight back to satellite. `LINK_TN_RECOVERY_DELAY_S = 300` |
| 1560 | TN | **Committed to cellular.** Probe backoff resets to 900. Steady state. |

End to end: roughly **5 minutes to fail over**, **26 minutes to come back** — and the asymmetry is
deliberate. Failing over is cheap and reversible; returning early to a marginal cell costs a
second failover.

---

## Interaction traps

Each is a pair of settings whose individually-reasonable defaults combine badly. They are the
reason this document exists.

### Probe cost — a failed probe is expensive under `SIMPLE`

Returning to satellite after a probe means a full re-attach. Worst case that is
`PROBE_TIMEOUT_S` plus `ATTACH_TIMEOUT_NTN_S` — around **18 minutes of disruption for one probe
that found nothing**.

Under `SIMPLE`, keep `PROBE_INTERVAL_S` well above 900. This is also the strongest practical
argument for `KEEP_CONTEXT`, and the number worth watching first in field data.

### Search timing — `PROBE_TIMEOUT_S = 180` is probably too short here

PLMN selection optimization has to stay disabled for Skylo to register, and that slows terrestrial
searches as well. Budget 240–300 s.

Getting this wrong produces the worst available failure mode: probes fail in places that *do* have
coverage, and the logs look exactly like genuinely absent coverage.

### Detection lag — `LINK_FAIL_SENDS` is gated by `TEST_INTERVAL`

At a 120 s send cadence, three failures takes six minutes — longer than the 120 s loss delay, so
registration state will essentially always trigger first.

That is acceptable, because this input exists only for the case registration cannot detect:
attached and registered, but the bearer silently dead. Just do not expect it to be quick, and do
not tune the loss delay assuming it contributes.

### Attach patience — `ATTACH_TIMEOUT_NTN_S = 900` may be optimistic

Nordic's *NTN operation v1.4* notes that the first satellite pass often draws an attach reject,
with acceptance only on a later pass. If timeouts show up under known-good sky view, this is the
number to raise before suspecting anything else.

### Position freshness — location validity gates every return to satellite

`NTN_LOCATION_VALIDITY_S = 3600` means that after an hour on cellular the modem's position has
expired, and the next failover must wait out a GNSS fix before it can begin.

GNSS coexists with terrestrial mode, so that fix is *free* while on cellular. Refresh it
opportunistically there and failover never pays for time-to-first-fix. This one only becomes
visible when you read the two settings together.

### Retry overlap — `NO_SERVICE_RETRY_S` should exceed the attach timeouts

At 300 s against a 300 s terrestrial attach timeout, a retry can fire while the previous attempt
is still timing out. Use 600.

---

## Presets

Three coherent sets rather than a shopping list. Bench values are deliberately impatient so a full
failover-and-recovery cycle completes in a couple of minutes with the override jumpers fitted.

| Symbol | Bench | Drive test | Deployment |
|---|---:|---:|---:|
| `LINK_TN_LOSS_DELAY_S` | 30 | 120 | 300 |
| `LINK_TN_RECOVERY_DELAY_S` | 60 | 300 | 600 |
| `LINK_TN_PROBE_INTERVAL_S` | 120 | 900 | 3600 |
| `LINK_TN_PROBE_TIMEOUT_S` | 180 | 300 | 300 |
| `TEST_INTERVAL` | 30 | 120 | 600 |
| `LINK_OVERRIDE_GPIO` | `y` | `y` | `n` |

Bench keeps the overrides on; drive test keeps them available for reproducing a fault on the
roadside; deployment compiles them out.

---

Defaults in this document are proposals, not measured optima — particularly the probe and attach
timings, which depend on the terrestrial coverage and satellite pass geometry at the test site.
Expect the first field session to move at least the probe timeout and the satellite attach
timeout. Behaviour described here reflects `mfw_nrf9151-ntn_1.0.0` and NCS v3.3.1; the constraint
that satellite and terrestrial system modes are mutually exclusive is a modem limitation, not an
application choice, and everything above follows from it.
