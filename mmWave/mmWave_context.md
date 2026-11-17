# mmWave (LD2450) Door Counter — Context

Reference notes for the LD2450-based people counter in `mmWave/`: hardware setup,
how the code works, how to operate it, known limitations, and history.

## Files

| File | Role |
|---|---|
| `mmWave.ino` | Sketch: pins, door geometry, LED blink, BOOT-button controls, prints |
| `LD2450.h/.cpp` | Radar driver: turns UART bytes into up to 3 `(x, y, speed)` targets per frame |
| `DoorCounter.h/.cpp` | Counting logic: tracks people across frames and detects door crossings |

Data flow:

```
LD2450 --bytes--> LD2450 driver --3 (x,y) per frame, ~10/s--> DoorCounter --ENTERED/EXITED--> mmWave.ino (LED, Serial)
```

## Hardware setup

- **Board:** classic ESP32 dev board (CP2102 USB chip, shows up as `/dev/cu.usbserial-0001`).
- **Radar wiring (soldered):**

  | LD2450 | ESP32 |
  |---|---|
  | TX | GPIO 3 (RX0) |
  | RX | GPIO 1 (TX0) |
  | GND | GND (**must be shared** with the ESP32 if the radar runs on a separate battery) |
  | 5V | 5 V supply (battery or ESP32 5V/VIN pin; not 3V3) |

- **LED:** GPIO 2 (on-board). **Button:** GPIO 0 (BOOT).
- **Mounting:** on the front of the top of the door frame, antenna pointing down, rotated so the
  radar's X axis runs in the walking direction (perpendicular to the door). Y is then the distance
  straight down to the person. The doorway is the line X = 0 directly under the sensor; the room
  is +X by default.

### Consequences of using GPIO 1/3 (shared with USB)

- The radar is read on UART2 with RX = GPIO 3 at 256000 baud; UART0 keeps GPIO 1 for Serial
  Monitor output at **115200**. `radar.begin()` must come after `Serial.begin()`.
- The Serial Monitor **cannot send input** to the ESP32, so controls are on the BOOT button.
- The ESP32 **cannot send commands** to the radar (no TX pin). Not needed: the LD2450 defaults
  to multi-target mode.
- The radar receives the debug text on GPIO 1 and ignores it.
- **Unpower the radar while uploading.** Its TX fights the USB chip on GPIO 3, causing
  `Failed to connect` / `Invalid head of packet (0xFF)` errors. If it still fails while unpowered,
  add a jumper or a 1 kΩ series resistor in the radar TX ↔ GPIO 3 wire.
- Moving the radar to GPIO 16/17 would remove all of these issues (one-line code change:
  `RADAR_RX_PIN 16`, `RADAR_TX_PIN 17`).

## Operating it

- Serial Monitor at **115200** baud. Press **EN** to see startup lines.
- **BOOT button:**
  - tap (< 1 s): reset counts
  - hold 1–3 s: toggle the live printout (`radar: [x,y speed] tracks: #id(x,y side)`)
  - hold > 3 s: swap which side is inside (also resets counts)
- **LED:** one long blink = entered, two short blinks = exited.
- Event line format:
  `[Track 7] ENTERED (X: 412 mm, Y: 950 mm) | In: 3  Out: 2  Occupancy: 1`
- Setup check: turn on the live printout, walk through the door, confirm X changes sign as you
  pass under the radar. If entries and exits are reversed, hold BOOT > 3 s.

### Troubleshooting messages

| Message | Meaning |
|---|---|
| Startup lines then silence | Normal: frames arriving, nobody crossing yet |
| `No data from LD2450 ...` every 2 s | No bytes on GPIO 3: power, missing shared GND, or wiring |
| `LD2450 sending data (N bytes) but no valid frames` | Bytes arrive but are garbled: bad ground or wrong baud |

### Toolchain notes (Apple Silicon Mac)

- The Arduino IDE's bundled `ctags` is Intel-only; Rosetta 2 was installed to fix
  `bad CPU type in executable`.
- `Wrong boot mode detected (0x13)` on upload: hold BOOT during `Connecting...`, lower
  Upload Speed to 115200 if needed.

## How the code works

### Driver (`LD2450.cpp`)

- Frame (30 bytes, ~10 Hz): `AA FF 03 00` | target1 (8B) | target2 (8B) | target3 (8B) | `55 CC`.
- Each target: X, Y, speed, resolution as little-endian 16-bit values. X/Y/speed use
  sign-magnitude with **bit 15 set = positive** (`decodeSigned`). All-zero block = empty slot.
- `feed()` is a byte-at-a-time state machine: discard until the 4-byte header, collect 26 more
  bytes, accept only if the footer is `55 CC`. Never blocks; handles frames split across reads.
- API used by the sketch: `radar.update()` (true when a new frame arrived) and
  `radar.target(i)` → `{valid, x, y, speed, resolution}` in mm / cm/s.
- Also contains configuration commands (firmware version, single/multi-target mode, zone
  filter, restart, factory reset). These are unused with the current wiring (no TX pin).

### Counter (`DoorCounter.cpp`)

Each person is a **track** that remembers which side of the door they are on. Per frame:

1. **Filter:** keep detections with X in [-2500, 2500] mm and Y in [150, 2500] mm.
2. **Match:** repeatedly pair the closest (track, detection) within the gate. Gate = 500 mm +
   3000 mm/s × time since last seen (re-acquires briefly lost people).
3. **New tracks:** unmatched detections start new tracks; their starting side is set
   immediately and the first sighting never counts.
4. **Expire:** delete tracks not seen for 1 s.

Crossing rule (`stepSide`):

- `sideOf` → −1 outside, 0 in the ±150 mm dead band, +1 inside.
- In the dead band or on the same side → nothing.
- On the other side for **2 consecutive frames** (`confirmFrames`) → switch side and emit
  `ENTERED` / `EXITED`. Occupancy never goes below 0.

Tunable settings (`DoorCounter::Config`, set in `setup()`): `lineMm`, `hysteresisMm`,
`insideIsPositive`, ROI bounds, `confirmFrames`, `trackTimeoutMs`, `gateMm`, `maxSpeedMmPerS`.

### Sketch (`mmWave.ino`)

- `setup()`: geometry config, register `onDoorEvent`, open radar UART.
- `loop()`: button, LED blink (non-blocking), `radar.update()` → `counter.update()`,
  no-frames warning after 2 s.
- WiFi/HTTP posting exists but is disabled (`ENABLE_WIFI = false`).

## Milestone test scenario

Simulated (host-side test with fake frames, including a dropout directly under the radar and a
3 m/s exit) and all steps matched:

| Count | Action |
|---|---|
| 0 | Start |
| 1 | Walk in |
| 2 | Walk in |
| 1 | Walk out |
| 1 | Walk part way in then walk out |
| 1 | Walk part way out then walk in |
| 2 | Walk in towards one end of the door |
| 1 | Walk out quickly |

Part-way moves stay correct either way: past the dead band they count in then out (net zero);
inside the dead band nothing counts. Not yet validated on real hardware end-to-end.

## Known limitations

- **Side-by-side people merge into one.** The LD2450 measures only range and one angle. With the
  overhead mount the angle runs along the walking direction, so it cannot see position across the
  door width. Two people side by side → undercount by one.
- **Tailgating:** one behind the other ≳ 0.7 m apart should count both; < ~0.5 m likely merges.
- **Opposite directions passing:** may merge briefly; net occupancy usually still correct.
- **Standing still in the doorway > ~1 s:** radar mostly detects motion; the track can expire and
  the crossing can be missed.
- **Errors persist:** a missed count is not self-corrected; occupancy is clamped at 0.

### Mitigation ideas (final design plan)

1. **Split inheritance (software):** when a new track spawns next to an existing one near the
   line, inherit the parent's starting side so both people count after separating.
2. **Side mount at waist height** looking across the door: side-by-side people differ in range
   (~0.5 m, near the radar's resolution limit).
3. **Thermal fusion (MLX90640):** overhead view shows separate warm heads; if radar sees one
   track but thermal sees two blobs crossing, count two. Also rejects radar false positives.

## MLX90640 thermal camera (planned fusion)

- **Lens: BAB, 55° × 35°** (confirmed by the field-of-view test in `mlxCamera/mlxCamera.ino`).
- Wiring: SDA → GPIO 21, SCL → GPIO 22, 3.3 V, GND, I2C address 0x33.
- `mlxCamera/MLX90640_I2C_Driver.cpp` implements the Melexis API's I2C functions with `Wire`.
- Coverage from the top of the door frame (~2.1 m): head height 0.42 × 0.25 m, shoulders
  0.73 × 0.44 m, floor 2.2 × 1.3 m (long × short axis).
- Plan: long (32 px) axis across the door width; use it to count simultaneous people (hot peaks),
  let the radar decide direction. Use refresh `0x05` (≈8 full frames/s) and 400 kHz–1 MHz I2C.
  Mounting higher (ceiling, 2.5–3 m) would enlarge the view substantially.
- Pipeline: background model (selective update) → threshold (+hysteresis) → connected
  components → peaks per blob → centroids → tracking/crossing (DoorCounter logic, fed points).

## `mmWave_counting` branch (teammate rjbert) — review notes

- `basic.ino`: same GPIO 1/3 wiring, but re-begins `Serial` at 256000 (Serial Monitor at 115200
  shows garbage), counts on X crossing 0 with no dead band (jitter double-counts), and the
  connection check always reports success (`uint8_t >= 0`, plus inverted checks).
- `mmWave and IR.ino`: radar on GPIO 16/17 (wrong for our board), requires thermal confirmation
  for every crossing (any warm blob anywhere confirms), 1.5 s cooldown breaks "part way in then
  out", and has the stale-slot bug.
- **Stale-slot bug:** state is stored per radar slot (slot 1/2/3) and never cleared when a person
  leaves. The next person assigned that slot on the opposite side produces a phantom crossing.
  `DoorCounter` avoids this by tracking people by position and expiring tracks after 1 s.
- Library there (ncmreynolds-derived) parses whole buffers and drops frames split across reads.
- Good ideas worth keeping: thermal confirmation, thermal blob count (computed but unused),
  LED feedback.

## Possible simplifications (not done)

Safe cuts for the current setup: radar command code in `LD2450.cpp/.h` (~110 lines, unusable
without a TX pin), `Config.axis` / `AXIS_Y`, the unused return value of `DoorCounter::update()`,
WiFi code until the networking milestone. Keep nearest-neighbour matching and
`confirmFrames = 2`; they prevent phantom and double counts.

## Next steps

- Validate the test scenario on the real door frame; record how close tailgaters can be.
- Add split inheritance and re-run the simulated scenarios.
- Add MLX90640 thermal fusion as an optional confirmation/second-person check.
- Prepare mounting diagram, final-design plan, and bill of materials for the milestone.
