# Speed Filter Review

## Symptom Observed

`make serial` reproduced the rest-state spike. The output was mostly `Speed: 0` or `Speed: 1`, then one sample printed:

```text
Timestamp(1hz): 186000
CAN error count: 0
CAN last error: 0x00000000
Speed: 6369
Throttle value: 32767
```

The project reports speed in `cm/s`, so `6369 cm/s` is about `142 mph`. With the current conversion constants, that corresponds to a Castle raw speed register around `12556`, which is a plausible 16-bit value and not a sentinel.

## Code Path Reviewed

- `board/main.c`: 20 Hz loop assigns `speed = read_speed()`.
- `board/castle.h`: `read_speed()` reads Castle register `Speed`, converts electrical RPM to vehicle speed, and returns `uint16_t cm/s`.
- `board/can.h`: CAN TX snapshots the current `speed` and sends it little-endian on CAN ID `0x209`.
- `docs/castle_serial_link_protocol.md`: Speed register is motor electrical RPM, register conversion is `Register / 2042 * 20416.66`.

## Findings

1. No obvious torn local `speed` variable access.

   `speed` is a `volatile uint16_t`. It is written and read from the main loop paths, not from the CAN RX interrupt. On Cortex-M, an aligned 16-bit load/store should not tear in normal memory, and the observed UART print comes from the same main loop that assigns speed. A torn global `speed` does not look like the primary failure mode.

2. I2C read framing mostly matches the local Castle protocol doc.

   The command packet includes address/register/data/checksum, but HAL sends the slave address separately, so transmitting `reg, 0, 0, checksum` while computing checksum over the address byte is reasonable. The read response is 3 bytes: high, low, checksum. The code validates `checksum == 0 - (high + low)`, which is consistent if the response checksum covers only those response bytes.

3. A valid-but-wrong Castle speed sample is accepted immediately.

   `read_i2c_reg()` only checks transport success and additive checksum. `read_speed()` only rejects `0xFFFF`. Any other raw value, including an isolated high RPM value while stopped, is converted and published. The serial sample had no CAN errors and no printed I2C error callback, so the spike appears to have passed the current checks as ordinary data.

4. Read failures collapse speed to zero.

   `read_speed()` returns `0` on any I2C read failure. That is safe for actuation consumers, but it loses diagnostic distinction between "vehicle stopped" and "telemetry failed". It may also hide intermittent read failures in the existing 1 Hz debug print.

5. The current checksum is weak for semantic/data-source faults.

   An 8-bit additive checksum can reject many wire errors, but it cannot prove the Castle register sample is physically plausible. It also cannot detect a sample that the Castle Serial Link itself produced from internally inconsistent high/low bytes before computing its own checksum.

## Likely Root Cause

The most likely path is a bad raw Castle electrical-RPM sample near rest, not a torn `speed` variable in ECU memory. It could be generated inside the Castle ESC/Serial Link telemetry path, or by an I2C transaction that still returns a self-consistent data/checksum tuple.

## Filter Recommendation

Do not rely only on an absolute max-speed cap. A `100 mph` cap would reject the observed `142 mph` spike, but it would still accept an isolated `60 mph` or `90 mph` sample while the car is sitting still. Use the cap as a final sanity guard, and use a continuity filter as the main protection.

Recommended first pass:

- Hard reject anything above `100 mph` (`4470 cm/s`) because it is outside the expected operating envelope.
- When filtered speed is near stopped, require two consecutive nonzero/high samples before allowing a jump out of zero.
- Apply a per-sample acceleration limit at the 20 Hz read rate so speed cannot jump by thousands of `cm/s` in one 50 ms update.
- Allow speed to decrease faster than it increases so bad high samples decay quickly and stopping behavior is not delayed.

This is still simple, but it catches both classes of bad data: absurd top-end values and plausible-looking values that are impossible given the previous sample.

## Gameplan

1. Add raw speed diagnostics first.

   Temporarily log accepted raw speed register, converted speed, checksum failures, read failures, and Castle `CheckBad`/`PacketBad` counters. This will show whether spikes are raw register spikes and whether Castle itself reports bad packets.

2. Change `read_speed()` to preserve last good value on I2C failure.

   Return status separately from value, or add a small stateful wrapper so transport failures do not look identical to true stopped speed. For CAN, publish the last good filtered speed or zero after a timeout/stale threshold.

3. Add a near-zero spike rejector.

   If the last filtered speed is below a small stopped threshold, reject a single sample above a high threshold unless it repeats. Example: while filtered speed is `< 20 cm/s`, require two consecutive samples before accepting `> 200 cm/s`.

4. Add acceleration-based plausibility limiting.

   At 20 Hz, cap per-sample speed increase to a physically plausible delta. For this RC platform, start conservatively around `100-200 cm/s` per 50 ms and tune from logs. Allow faster decreases than increases so speed can drop safely.

5. Use a small median or repeated-sample filter.

   A 3-sample median at 20 Hz adds only 50-100 ms of latency and kills isolated single-sample spikes. This is a good first implementation because the observed failure is isolated and large.

6. Keep units explicit.

   Name internal variables `raw_electrical_rpm_reg`, `speed_cm_s`, and `filtered_speed_cm_s` so the conversion and CAN contract stay clear.

## Proposed Implementation Shape

- Keep `read_i2c_reg()` as the low-level transport primitive.
- Add `read_speed_raw(uint16_t *raw)` for diagnostics and clearer separation.
- Add `speed_filter_update(uint16_t sample_cm_s, bool sample_valid)` with static state:
  - reject invalid transport samples without immediately forcing output to zero;
  - reject isolated near-zero-to-high spikes;
  - median-of-3 accepted samples;
  - acceleration clamp on the filtered output.
- In `main.c`, replace `speed = read_speed();` with a status-aware update:

```c
uint16_t sample_cm_s = 0;
bool sample_valid = read_speed_cm_s(&sample_cm_s);
speed = speed_filter_update(sample_cm_s, sample_valid);
```

## Initial Implementation

Implemented on branch `speed-filter`:

- `read_speed_cm_s()` returns a validity status instead of folding I2C/sentinel failures into `0`.
- `speed_raw_to_cm_s()` keeps Castle raw register conversion separate from filtering.
- `speed_filter_update()` applies:
  - hard reject above `100 mph` (`4470 cm/s`);
  - hold-last-good behavior for invalid samples, falling back to zero after `500 ms`;
  - two-sample confirmation before jumping from near-rest to above the stopped threshold;
  - rise-rate limit of `200 cm/s` per 20 Hz sample.
- Hardware testing caught isolated `200 cm/s` samples at rest, so the zero-exit confirmation was tightened from `> 200 cm/s` to any sample above the stopped threshold.
- `main.c` now assigns `speed = read_filtered_speed()` in the 20 Hz loop.

Deferred until hardware testing:

- Raw rejected-sample counters/prints.
- Median-of-3 filtering, if the first-pass continuity filter still lets smaller spikes through.

## Verification Plan

1. Build with `make`.
2. Run `make serial` at rest for at least 2-3 minutes.
3. Confirm no published speed spikes above the stopped threshold.
4. Temporarily print rejected raw samples to verify the filter is catching the existing failure.
5. Roll the car by hand or drive slowly and confirm low-speed response is not pinned at zero.
6. Drive a short controlled run and tune acceleration thresholds from observed real data.
