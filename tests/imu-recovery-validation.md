# IMU recovery validation

The implementation covers the firmware, configuration migration and static web UI
in `docs/imu-recovery-implementation-plan.md`. Physical acceptance remains pending.

## Isolated firmware tests

`tests/imu_recovery` is an ESP-IDF v5.3 Unity application for ESP32-S3.
It links production IMU worker/FIFO/estimator/calibration, motor gate, state manager,
fault dispatcher, balance monitor and configuration parser/validator code.
Only the I2C device, IRQ and motor driver implementations are replaced.
The test component uses `WHOLE_ARCHIVE` so every Unity registration is retained.

From an exported ESP-IDF shell:

```text
cd tests/imu_recovery
idf.py -D IDF_TARGET=esp32s3 -D SDKCONFIG=build/sdkconfig build
```

The separate test configuration uses a 1 ms RTOS tick and disables exceptions/RTTI.
The test application starts the Unity menu. Build validation does not flash or run it.

Cases cover:
- Two-motor commit/stop serialization, zero output for disabled/non-finite requests,
  expiry checked inside the output lock, revoked arms and delayed enable/fault events.
- A dedicated fault latch with a saturated ordinary event queue.
- Fresh auto-balance holds, duplicate samples, gaps and obsolete availability events.
- No latent start and fault exits from balancing, tuning and guided calibration.
- FIFO failure after zero, partial or complete byte consumption without retry/publication;
  bounded count retry, packet remainder, conservative timestamps and resync errors.
- Invalid boot/reset estimates, finite checks, sequence/generation continuity and sample age.
- Single hardware owner, IRQ polling fallback, deferred configuration, cooperative shutdown,
  absent-sensor reconnect cadence and validation timeout.
- Failed FIFO repair escalation to full reconnect.
- Calibration cancellation/disconnect, motion/OTA exclusion and preservation of old offsets.
- Legacy configuration migration, future versions, malformed new fields and timing rejection.

Tests use fake devices but run under FreeRTOS. Assertions have **not been executed**
on an ESP32. Use a runner timeout for concurrency tests when execution is authorized.
Scheduler starvation, real GPIO/LEDC/I2C errors, heap stability and physical cutoff
latency require the on-device acceptance procedure; compiled tests do not prove them.

## Web tests

Serve the repository root with a local HTTP server, then open
`tests/web/imu-status.test.html`. Alternatively, with Node.js:

```text
node -e "import('./tests/web/imu-status.test.js').then(async m => console.log('PASS:', await m.runTests(), 'checks'))"
```

The tests exercise production decoder, state, telemetry, API and status rendering:
legacy/unknown data, version 2 metadata, invalid gaps, generation boundaries,
preserved independent sensor history, malformed/future formats and fetch failure.

## Validation record — 2026-09-07

- Full firmware build passed using ESP-IDF v5.3 and the existing ESP32-S3 configuration: `0x1123d0` bytes, 14% app partition space free.
- Isolated Unity application built and linked successfully: `0x53f40` bytes, 22 registered cases. Registration retention was checked in the linked image.
- Web suite passed all 15 checks both in Node.js and in the in-app browser.
- Syntax and relative imports passed for all 16 JavaScript modules; `git diff --check` passed.
- Configuration differences were checked without printing values; the existing user edit remains alongside the IMU migration.
- Existing LEDC initializer warnings remain.
- No board flashing, runtime Unity execution, motor tests, timing measurements or soak
  tests were performed.

Hardware acceptance follows section 9.3 of the implementation plan, beginning with
motor power disconnected. Measure the 20 ms sample-age / nominal 25 ms cutoff target,
then absent-device recovery, fault injection under load and long-duration operation.

## Regression correction after device feedback

The device log reported repeated GPIO setup and IMU resets roughly every 1.2 seconds.
The prior build-only result did not establish working balancing. FIFO production
started before slow IRQ/GPIO setup; the conservative oldest-packet age rule rejected
that startup backlog and repeated the process during resync.

The revision moves setup before FIFO enable, reads queued data before stale-snapshot
escalation, removes the extra age period for complete packets, uses newest bounded-batch
age, reads below the configured threshold when necessary to preserve freshness,
and restores the original corrupt-packet guard. Graph history now survives generations
with a local gap. No sample-age limit or PID setting was increased to mask failures.

Five added embedded regression cases cover timestamp bias, a threshold near the deadline,
a fresh tail behind an old packet, 50 ms GPIO setup and corrupt zero/ones packets.
They are compiled, not executed. The revised web suite passed 20 checks in Node.js
and the browser, including history retention within and between responses.
Corrected on-device behavior is not yet confirmed; no flash or motor test was performed.

Final regression build: firmware `0x112480` bytes (14% free); Unity application 0x54940 bytes, 27 registered cases confirmed in the image. Both builds passed; `git diff --check` and JavaScript syntax checks passed.

## 2026-09-08: failure at balancing entry

The new device log showed a roughly 30 ms balancing-to-idle transition after the
upright hold, followed by FIFO repair. The old log did not include the FIFO fault
reason. Code review found logging under RobotController's control-mode mutex and
age-only FIFO repair still present.

The update removes that mutex-held log, downgrades routine command mode/timer logs,
and drains ordered old FIFO samples without treating age as lost framing. Remaining
backlog causes a one-tick worker wait. A 250 ms acquisition no-progress deadline is
separate from the unchanged 20 ms motor freshness deadline. Overflow, corrupt packets
and uncertain destructive reads still request repair. Fault diagnostics now report
control stage, original/latest sample ages, motor error and FIFO repair reason.

Added embedded cases: old ordered backlog, an 80 ms acquisition gap without reset,
and sustained no-progress recovery. They compile but have not run on an ESP32.

Validation: firmware and isolated Unity builds passed; 30 cases retained in the binary;
20 web checks passed in Node.js; git diff whitespace validation passed. No hardware
execution was performed, so stable balancing remains unconfirmed on the device.

Build sizes: firmware 0x1124b0 bytes; Unity 0x552d0 bytes.

## 2026-09-08: PCNT and coherent sensor frames

The user now reports that the preceding IMU correction appears to work. This is
user feedback, not completion of the instrumented cutoff/soak checklist. The review
and remaining limitations are in `docs/imu-pcnt-sensor-review.md`.

Implemented PCNT high/low watchpoints, checked initialization, removal of incorrect
wrap arithmetic, measured encoder intervals, invalid-read handling, int64 logical
counts and stopped rebasing before the SDK signed accumulator approaches overflow.
PCNT maintenance continues when telemetry is disabled. Very short catch-up iterations
retain the previous timestamped frame rather than amplifying a single pulse.

SensorFrame now publishes acceleration, ADC saturation, FIFO backlog/loss metadata
with the existing orientation and validity. EncoderFrame publishes both wheels under
one cross-core lock. Controller gates all active modes on both wheel validity flags
and uses the oldest input timestamp at the final motor commit. Telemetry v3 appends
wheel validity and IMU sample reuse; v2/legacy readers remain supported by the UI.

Validation performed:
- Full ESP32-S3 firmware build passed: 0x113090 bytes, 14% of the app slot free.
- Isolated Unity build passed: 0x57ff0 bytes, all 41 test names retained in the image
  with WHOLE_ARCHIVE. Production PCNT/motor hardware drivers are replaced by fakes.
- Eleven additional cases cover both-wheel/both-direction limit crossings, all PCNT
  setup failures, read failures, real measurement periods, rebase clear/stop/start
  failures, large discontinuities, sub-millisecond reuse, two cross-core frame readers,
  loss metadata and ADC rail decoding. These are compiled, NOT executed on an ESP32.
- Web suite: 24 checks executed successfully in Node.js, including v3 validity and
  malformed payloads while preserving v2/generation-gap behavior.
- All 16 web module syntax/relative-import checks and git diff whitespace validation passed.
- No flashing, monitoring, motor operation, runtime Unity/concurrency execution or
  physical timing/soak measurements were performed.

Additional hardware acceptance after authorization:
1. Run Unity including [encoder] and [concurrency]; retain actual pass/fail output.
2. Cross +30000 and -30000 on each wheel in both directions, also reversing around
   the limit, under ordinary web/IRQ load. Observe no sign reversal or speed impulse.
3. Record rawCount, deltaCount, measurementPeriodUs, validity and maximum speed;
   compare the maximum delta with the measured physical wheel speed (not an assumed
   motor rating). The fake limit test expects 7 pulses/5 ms = 180 degrees/s.
4. Check constant wheel setpoints over repeated limit crossings, then the original
   IMU cutoff and soak checklist. A correct count trace with continued overshoot
   warrants examining PID integral/output saturation; PID gains were not changed.

The large-delta guard is half the smaller PCNT limit, not a calibrated physical
speed ceiling. The motor rating/maximum wheel speed was requested but remains unknown.
A stopped rebase marks continuityLost because edges during the stopped interval are
unknown; its invalid frame inhibits motion and requires the normal fresh arm policy.


## Sporadic jerk follow-up: control timing, encoder filtering and PCNT ISR observation

Device feedback reports disappearance of the previous runaway/settling pattern but
occasional jerks. A contemporaneous state/fault log is still unavailable; the actual
on-device trigger is not proven.

Changes:
- ControlTask reanchors each deadline at the actual wake tick and waits before its
  first step; missed deadlines are not replayed as rapid PID iterations. PID receives
  measured elapsed time, including intervals previously replaced with nominal dt.
- Encoder filter alpha scales with elapsed time using log1p/expm1, referenced to the
  configured MainLoopConfig period supplied by ApplicationContext. Nominal-period
  response and alpha endpoints remain unchanged; no PID defaults/equations changed.
- A half-limit PCNT discontinuity gets one immediate, nonblocking re-read to handle
  observation between hardware reset and accumulator ISR. Persistent jumps and read
  failures still invalidate feedback and inhibit motion. No count correction is inferred.

Validation:
- Firmware build passed: 0x113700 bytes, 14% app slot free.
- Unity build passed: 0x59620 bytes; all 48 registrations retained. Seven new cases
  cover nominal/jittered filter response, single short-interval pulse, alternate
  nominal periods/endpoints, delayed schedule, tick wrap, production PID derivative
  response and both-direction/both-wheel PCNT observations before accumulator ISR.
- 24 web checks executed successfully in Node.js; git diff --check passed.
- Unity cases were compiled, NOT executed. No board flashing, monitoring, motor
  operation or physical timing measurement was performed.

On-device follow-up should distinguish a brief effort impulse while staying in
BALANCING from an IDLE transition accompanied by 'Control inhibited: invalid-encoder'
(or another cause). Observe the unchanged PID settings first; preserve all freshness
and arm-revocation gates. The stored speed PID integral range exceeds its output range;
assess saturation/windup using actual traces rather than assume that retuning explains
new isolated jerks. Details: docs/imu-pcnt-sensor-review.md.
