# NUFLIGHT v2 combined firmware/decoder standard

The CM7 logger records live LSM6DSO telemetry, GPS NAV-PVT telemetry, and the complete `navBus` (`navigation_bus`) and `controllerBus` (`controller_bus`) produced by `autocodeModel`. The plant and Plant-to-Model simulation are disabled. The CM7 build creates a separate `libautocodeModel_CM7.a` from the primary model export and its runtime dependencies, excluding all plant sources.

The generated sensor bus has IMU, magnetometer and barometer fields, but no GPS input. GPS is logged independently alongside model outputs. Hardware barometer acquisition stays commented out; `baroMeas_Pa` is exactly **1000.0 Pa** both during initialization and every sensor-bus update. `bmp585_pressure` records that same placeholder. Unacquired BMI, BMP581, temperature and magnetometer values remain zero. LSM gyro degrees/s are converted to rad/s before entering the model.

The first file is `flight.bin`; existing logs are preserved using `FL000001.BIN` and the next available number. Sampling is nominally 100 Hz, continuously. Every scheduled sample acquires the IMU, maps the current sensor input, and executes one model step. TIM5 drives sampling and model updates. While logging is active, the ISR queues each complete record in a 128-record RAM queue. The foreground encodes and writes records, and handles FatFs sync and close. If the queue is full, the producer drops the newest record and increments telemetry_queue_drops. Packet sequence gaps expose these losses; the actual interval between stored acquisitions is `timestamp_us_delta`; sum deltas with a wide/unbounded host integer to obtain elapsed time. Model step-duration/deadline counters remain debugger-visible. No PWM/actuator output is enabled.

GPS uses USART2 at 9600 baud, 8N1, a 1024-byte circular DMA receive buffer on DMA1 Stream 0, and an asynchronous NAV-PVT poll once per second. The TIM5 sample service drains the ring and checks UBX checksum and length. The DMA completion callback counts wraps so the reader can detect a full-ring overrun. The reader discards the backlog and resynchronizes after an overrun, and restarts DMA after HAL reports a receive error. Logging never waits for a satellite fix. Latitude/longitude are degrees, altitude is meters above the ellipsoid, and velocity N/E/D is m/s. `gnssFixOK` stores the latest receiver NAV-PVT flags bit 0 as 0/1 (zero before the first message). `gps_valid` requires gnssFixOK and a message no older than 2000 ms; this is the validity supplied to model input `gpsFix_b`. `gps_age_ms` is UINT32_MAX until the first NAV-PVT message; latest coordinates are retained even if the fix is subsequently invalid, so check validity/age.

## LOCKOUT logging tail and timestamp intervals

The first sample with `nav_flightMode_enum == LOCKOUT` (3) latches HAL_GetTick. Logging continues until 120000 ms have elapsed from that observation, even if the mode later changes. The first scheduled service at or after the deadline stops adding records. The foreground drains the queue, syncs and closes the SD file, and reports `Logging stopped: 120 seconds after LOCKOUT`. The main loop, GPS receiver and model keep running. The tail timer is not reset by repeated LOCKOUT samples and unsigned subtraction handles HAL's millisecond tick rollover. The file close can lag the sampling cutoff while the foreground drains the queue and completes SD operations.

The first stored `timestamp_us_delta` is 0. Each subsequent value is `(uint32_t)(current_sample_timer_us - previous_written_sample_timer_us)`, normally about 10000. The internal 32-bit microsecond timer can roll over without causing a discontinuity in these short intervals. An interval of an entire 2^32 microseconds or longer cannot be represented uniquely; continuous 100 Hz operation does not approach that limit. The dynamic SD header names the field explicitly. Older headered logs with `timestamp_us` still describe an absolute timer and remain decodable; headerless legacy files retain their original timestamp_us interpretation.

## Boot and error status

The logger syncs the first packet and then once per second. `Logging started... <filename>` is sent on USART3 (PD8 TX, 115200 8N1) after the first successful record write and sync. The latest status is replayed on board USB CDC COM4 whenever the terminal opens/asserts DTR, after a nonblocking 300 ms settling interval. Sensor/SD initialization stages and failure outcomes are retained in the same way. SD/FatFs writes are bounded synchronous operations; the board is not claimed to have guaranteed storage latency.

## File structure

`header | packet[0] | packet[1] | ...`

Current header size is **1900 bytes** (67 descriptors). Current decoded payload size is **564 bytes**, and wire packet size is **643 bytes**. For N complete packets, file size is `1900 + 643*N`. The CSV contains 87 scalar columns. All integer metadata is unsigned little-endian. Payload IEEE-754 floats are little-endian.

### Fixed header prefix: 20 bytes

Python layout: `<8sBBBBHHHH`.

| Byte offset | Size | Meaning / v2 value |
|---:|---:|---|
| 0 | 8 | ASCII `NUFLIGHT` |
| 8 | 1 | Version = 2 |
| 9 | 1 | Endian identifier = 1 (little-endian) |
| 10 | 1 | Error-correction identifier = 1 (extended Hamming 72,64) |
| 11 | 1 | Integrity flags = 1 (CRC-32 present) |
| 12 | 2 | Total header bytes, including final CRC = 1900 |
| 14 | 2 | Unencoded payload bytes = 564 |
| 16 | 2 | Complete wire packet bytes, including marker = 643 |
| 18 | 2 | Number of field descriptors 67 |

The prefix is followed by `field_count` descriptors. Each is 28 bytes, Python `<24sBBH`: NUL-terminated ASCII name in 24 bytes, type ID (uint8), element count (uint8), byte offset into the decoded payload (uint16). Names must be unique after column expansion. Types cannot overlap or exceed payload bounds. The decoder reads the descriptor layout rather than assuming the current offsets below. Three-element fields expand to `_x`, `_y`, `_z`; other arrays use numeric suffixes.

Type IDs: 1=uint8 (`B`), 2=uint16 (`H`), 3=uint32 (`I`), 4=int32 (`i`), 5=float32 (`f`), 6=float64 (`d`). A version-2 file can describe a different payload layout using these types; packet size is derived from payload length.

A little-endian CRC-32 terminates the header. It covers all preceding header bytes (prefix and descriptors), excluding the CRC itself. Header corruption is detected, not corrected. A bad header is rejected rather than guessing field locations.

### Base telemetry prefix (the first 154 bytes of the combined payload)

| Offset | Field | Type | Count |
|---:|---|---|---:|
| 0 | timestamp_us_delta | uint32 | 1 |
| 4 | packet_counter | uint32 | 1 |
| 8 | bmi_accel | float64 | 3 |
| 32 | bmi_gyro | float64 | 3 |
| 56 | lsm_accel | float64 | 3 |
| 80 | lsm_gyro | float64 | 3 |
| 104 | bmp581_pressure | float32 | 1 |
| 108 | bmp581_temperature | float32 | 1 |
| 112 | bmp585_pressure | float32 | 1 |
| 116 | bmp585_temperature | float32 | 1 |
| 120 | latitude | float64 | 1 |
| 128 | longitude | float64 | 1 |
| 136 | altitude | float32 | 1 |
| 140 | velocity_north | float32 | 1 |
| 144 | velocity_east | float32 | 1 |
| 148 | velocity_down | float32 | 1 |
| 152 | gnss_fix_type | uint8 | 1 |
| 153 | gnss_satellites | uint8 | 1 |

Active LSM accelerations are m/s^2 and gyro readings are degrees/s in this base prefix. `timestamp_us_delta` is the uint32 interval since the previous successfully written SD packet; the first interval is zero. Internal timer subtraction is modulo 2^32, so a timer rollover is harmless when consecutive packets are less than 2^32 microseconds apart. Total run time is no longer stored in this field. `packet_counter` still wraps modulo 2^32. The base prefix retains its original packed layout.

### Complete model buses and GPS freshness

The combined packed record is:

| Offset | Member | Size |
|---:|---|---:|
| 0 | base (`FlightData_t`) | 154 |
| 154 | navBus (`navigation_bus`) | 248 |
| 402 | controllerBus (`controller_bus`) | 152 |
| 554 | gps_valid (uint8) | 1 |
| 555 | gps_age_ms (uint32) | 4 |
| 559 | gnssFixOK (uint8, raw latest receiver fix flag) | 1 |
| 560 | lsm_accel_samples (uint16) | 2 |
| 562 | lsm_gyro_samples (uint16) | 2 |

Generated bus members retain their ABI padding. Every semantic scalar/array member has its own descriptor with the actual byte offset; the decoder does not infer compiler padding. The ARM flight-mode enum uses uint8 storage, while another compiler can use int32; the descriptor selects the matching type. real_T values are float64 and boolean_T flags are uint8. Fields use these CSV aliases:

- `nav_*`: every navigation_bus member, including flightMode_enum, qEst_na[4], phi/theta/psi, p/q/r rates, accelerations, accelBias[3], gyroBias[3], altitudeBias_m, llaBias_na[3], dynamic pressure, velocity and position estimates.
- `ctl_o_{pitch,yaw,roll}_*`: all outer-loop rate commands, rateLimited and saturated flags.
- `ctl_i_{pitch,yaw,roll}_*`: all inner-loop control_deg, moment_Nm, rateLimited and saturated fields. The generated C member `rateLimted_b` is intentionally exposed as `rateLimited` in the CSV alias.
- `ctl_control_deg_0..3`: the four final controller commands.
- `gps_valid` and `gps_age_ms`: validity/freshness described above.

The refreshed November 3 generated model uses xVelEstNed_mps/yVelEstNed_mps/zVelEstNed_mps and includes altitude/LLA biases; all are described in the header. Compile-time bus-size assertions fail closed if a later export changes the bus layout.

Navigation and controller values are captured immediately after the model step driven by the same record's live IMU sample. There is no simulated plant step. The codec copies full bus structs and describes all fields; it does not truncate the buses to Euler angles or fin commands.

### Packet protection and framing

Each packet consists of ASCII `PKT2` (4 bytes), followed by 71 consecutive 9-byte SECDED codewords. For a general payload length P, the number of codewords is `ceil((P+4)/8)` and wire size is `4+9*ceil((P+4)/8)`.

The data before encoding is: 564-byte payload, its little-endian 4-byte CRC-32, with no padding, totaling 568 bytes. CRC and padding are themselves covered by SECDED encoding. Payload CRC covers only the 564 decoded payload bytes, including timestamp and packet counter.

For every 8 input bytes, extended Hamming(72,64) produces 9 encoded bytes:

1. Code positions are numbered 1 through 72. Position 1 is bit 0 (LSB) of encoded byte 0; position 72 is bit 7 of encoded byte 8.
2. Positions 1,2,4,8,16,32,64 are Hamming even-parity bits. Place the 64 data bits, input-byte order and LSB first within each byte, into the remaining positions 1..71.
3. Each Hamming parity bit makes its position-mask group even across positions 1..71.
4. Position 72 makes all 72 bits have even parity.
5. Decoder syndrome is XOR of all set positions 1..71. Overall parity is XOR of all 72 bits. Syndrome zero/even means clean; nonzero/odd means correct the indicated bit (1..71); zero/odd means correct bit 72; nonzero/even means detected uncorrectable error. A syndrome above 71 is rejected.

This corrects one bit per 72-bit codeword and detects every two-bit error within a codeword. Independent single-bit errors in multiple codewords can all be corrected. Errors involving three or more bits per codeword are not guaranteed correctable; CRC-32 rejects residual damage except for the normal CRC collision possibility. CRC is error detection, not authentication. The unencoded `PKT2` marker is for resynchronization, not ECC-protected; damage to a marker may lose that packet, but later packets can be recovered.

CRC-32 is CRC-32/ISO-HDLC (the zlib.crc32 convention): reflected polynomial 0xEDB88320, initial register 0xFFFFFFFF, final XOR 0xFFFFFFFF. Test vector ASCII `123456789` gives 0xCBF43926. Store the resulting CRC integer little-endian.

## Decoder

Updated installed script: `C:/Users/Christian/Documents/ActiveAero2027/activeControlSim/scripts/helpers/decodeFlightData.py`.

```powershell
py C:\Users\Christian\Documents\ActiveAero2027\activeControlSim\scripts\helpers\decodeFlightData.py -i D:\flight.bin -o D:\flight.csv
# Positional arguments are also accepted:
py C:\Users\Christian\Documents\ActiveAero2027\activeControlSim\scripts\helpers\decodeFlightData.py D:\flight.bin D:\flight.csv
```

With no input argument, the script prompts for input and output paths. With an input but no output argument, CSV defaults beside the input. `--force` permits replacing an existing CSV. Input and output cannot be the same path.

After reading the header, the decoder prints the detected version, byte order, header/payload/packet lengths, protection method, and every field's name, type, count and offset. It then writes the CSV and prints the output path and packet/correction/rejection counts. Correctable errors are repaired before decoding values. Uncorrectable packets are omitted with warnings, never emitted as valid telemetry. Marker search allows recovery after lost/inserted bytes. Partial packets at EOF are reported and omitted.

Exit codes: 0=conversion completed with no rejected/truncated packets or framing loss (corrected errors are allowed); 1=CSV produced with packet rejection/framing loss/truncation; 2=fatal error such as invalid header, filesystem failure or strict-mode rejection. `--strict` stops on the first rejected packet/framing error. A truncated tail is reported through exit 1.

For headerless files, automatic legacy detection recognizes the known 154-byte packed-double-vector layout when file size is a positive multiple of 154. It explicitly prints that integrity protection is absent. `--legacy` can force that known layout for a legacy file with a partial tail. Legacy files cannot self-describe other schemas, and the decoder does not guess older float-vector or padded variants. Apparent damaged NUFLIGHT headers are rejected instead of automatically treating them as legacy.

## Previous compact navigation UART protocol (NTL1 v2)

At the top of `CM7/Core/Src/main.c`:

```c
#define ENABLE_UART_TELEMETRY_PACKETS 1
```

Set to 1 for packet streaming, or 0 for status messages only. The committed default is 1. Packets go to physical USART3 TX PD8, 115200 baud, 8N1, no hardware flow control. The identical packet is mirrored over board USB CDC COM4 after DTR/port-open settling. Serial-library UART parity must remain None; the Hamming bits are part of the application packet. ASCII boot/stop/error messages share each transport between complete binary frames. The reader skips that text and recognizes the normal LOCKOUT stop line.

The raw packed C struct `UartTelemetryPayload_t` is 47 bytes, Python `struct.Struct('<BIIB9fB')`:

| Offset | Type | Field |
|---:|---|---|
| 0 | uint8 | version = 2 |
| 1 | uint32 | packet_counter (same sequence as SD record) |
| 5 | uint32 | timestamp_us_delta (same interval as SD record) |
| 9 | uint8 | nav_flightMode_enum |
| 10 | float32 | nav_phiEst_rad |
| 14 | float32 | nav_thetaEst_rad |
| 18 | float32 | nav_psiEst_rad |
| 22 | float32 | nav_xVelEstNed_mps |
| 26 | float32 | nav_yVelEstNed_mps |
| 30 | float32 | nav_zVelEstNed_mps |
| 34 | float32 | nav_xPosEst_m |
| 38 | float32 | nav_yPosEst_m |
| 42 | float32 | nav_zPosEst_m |

Angles are radians, NED velocity is m/s, position is meters. UART floats are converted to IEEE-754 binary32 to provide bandwidth margin; full SD navigation/controller values remain float64.

`gnssFixOK` is a uint8 at offset 46: the latest raw receiver fix flag, matching SD `gnssFixOK`. It is zero before the first GPS message; consult SD `gps_valid`/`gps_age_ms` for freshness. The reader also accepts older v1 packets and emits an empty/null `gnssFixOK` for those packets.

Append little-endian CRC-32 of the entire 47-byte raw payload, then five zero bytes. Encode those 56 bytes as seven Hamming SECDED(72,64) codewords using the same bit/parity convention as the SD codec. Prefix the 63 encoded bytes with ASCII `NTL1`. Complete packet length: **67 bytes**. The marker is for resynchronization; the version, sequence, time interval, mode, floats, CRC and padding are protected.

At 100 packets/s, 8N1 costs `67 * 100 * 10 = 67000 bit/s`, or **58.16% of 115200 baud**. Wire time is about 5.82 ms per packet, leaving 4.18 ms in each nominal 10 ms sample period. USART3 uses interrupt-driven transmission. Active transmit buffers are retained until completion. If the UART/USB endpoint is still busy or a status is pending, that transport skips the new packet rather than blocking acquisition or corrupting an active frame. Debugger-visible dropped-packet counters and receiver sequence gaps expose omissions; this bandwidth calculation is not a claim that SD stalls or radio links can never lose packets.

### Live decoder

Script: `C:/Users/Christian/Documents/ActiveAero2027/activeControlSim/scripts/helpers/readTelemetryUART.py`. It is self-contained for decoding; only live serial capture needs pyserial.

```powershell
py C:\Users\Christian\Documents\ActiveAero2027\activeControlSim\scripts\helpers\readTelemetryUART.py --port COM4 --output telemetry.csv
# For a physical USART3 adapter, replace COM4 with that adapter's COM port.
# Optional limits: --duration 180 or --count 10000. Ctrl+C also closes the CSV.
# Offline captured wire data:
py C:\Users\Christian\Documents\ActiveAero2027\activeControlSim\scripts\helpers\readTelemetryUART.py --input raw_uart.bin --output telemetry.csv
```

Existing CSVs require `--force`. The script saves every accepted packet to CSV and prints live values at most once per second. The columns match the raw fields above (except version) plus `ecc_corrected_bits`. It reports correction, rejection, duplicate, counter-reset, missing-sequence and partial-frame counts. It handles arbitrary input chunks, boot text, corrupted packets, and inserted/deleted bytes by resynchronizing to the next valid marker/CRC. Receipt of the exact LOCKOUT stop-status line ends capture automatically. Live capture stopped intentionally does not count its unread partial tail as corruption.

The UART delta is per SD sample, not necessarily per received UART frame. If the UART skips a sample, its sequence number jumps and that missing interval is unavailable; the reader reports the gap and does not fabricate elapsed time. Headered SD files remain the complete timing source when their packets are intact.

## Validation and deployment

Both ENABLE_UART_TELEMETRY_PACKETS=0 and =1 ARM builds passed. The packet encoder is absent from the disabled image. Final enabled image: text 141744 bytes, initialized data 744 bytes, BSS 77792 bytes. Plant simulation remains excluded.

Native tests used the actual firmware logger functions with mocked time/SD/UART drivers: first/normal/wrapped microsecond deltas, the exact 120 s cutoff across a millisecond rollover, a mode change after initial LOCKOUT, sync/close errors, and continued model execution after file close all passed. Tests also checked that a busy UART skips candidates without overwriting its active buffer.

The actual C UART encoder produced a fixture decoded in Python. All 504 protected bit positions were individually flipped and corrected; double-bit errors in each word were rejected. The parser passed boot-text, byte-chunk, duplicate, sequence-gap, counter-wrap, inserted/deleted-byte, partial-frame and CLI-to-CSV tests, including the device stop-status line. A C-generated SD fixture confirmed the existing dynamic decoder recognizes timestamp_us_delta and the complete bus schema.

D-cache remains disabled for SD IDMA buffers; the unaligned DMA scratch path is enabled. CM4 synchronization remains disabled. This build has not been flashed or hardware-tested; actual board/radio/SD timing and live UART reception remain connected-board checks.


## Current UART protocol: NTL3 v3

The current firmware sends marker `NTL3` and version 3. The packed payload is
59 bytes, Python `<BIIB9fB3f`: it retains all v2 fields at the same offsets,
then appends raw receiver velocity `gps_velocity_north_mps` at offset 47,
`gps_velocity_east_mps` at 51, and `gps_velocity_down_mps` at 55 (float32 m/s).
These are NAV-PVT velN/velE/velD converted from mm/s to m/s, without model filtering.
Latest values are retained between GPS messages, including after fix loss;
`gnssFixOK` remains the latest receiver flag, not an age/freshness indicator.

Append CRC-32 at offset 59 and one zero padding byte, giving 64 protected bytes.
Encode as eight SECDED(72,64) words and prefix the four-byte marker: 76 bytes total.
At 100 Hz, 115200 baud 8N1 uses 76000 bit/s (66.0%), with 6.60 ms wire time.
Transmission remains interrupt driven; busy streams drop new packets rather than wait.
The updated `readTelemetryUART.py` and `plotUART.py` accept both NTL1 v1/v2 and
NTL3 v3, including mixed streams. Missing GPS velocity fields in legacy packets
are null/empty CSV cells and N/A in the plot. CSV is disabled unless `--output` is passed.

`plotUART.py` adds a raw GPS velocity panel. Axes are fixed by default:
attitude [-pi, pi] rad, both velocity panels [-100, 100] m/s, position [-1000, 1000] m.
Override with `--angle-range MIN MAX`, `--velocity-range MIN MAX`, or
`--position-range MIN MAX`. Out-of-range traces are clipped with a visible warning;
latest numeric readings remain available. The horizontal window remains fixed in
span (`--window`, default 15 seconds) and scrolls using host reception time.


## IMU acquisition and model-window averaging

`IMURunningBuffer.c/.h` owns the six running sums, double-bank storage,
overflow limits, window counts and conversion to averaged FlightData_t fields.
The LSM6DSO driver initializes its conversion scales and feeds raw samples from
DMA callbacks through `IMURunningBuffer_AddAccel` / `IMURunningBuffer_AddGyro`.
`main.c` discards startup samples with `IMURunningBuffer_Reset` and calls
`IMURunningBuffer_ReadAverage` immediately before each model update.

Accelerometer and gyroscope ODR are explicitly 1667 Hz (the driver's name for
LSM6DSO's nominal 1.66 kHz mode). The model/logging schedule remains 100 Hz.
Both data-ready sources are routed as active-high pulses to both sensor INT pins;
the existing LSM_INT / PG9 EXTI starts acquisition. Runtime reads use SPI1 DMA,
DMA2 Stream0 RX / Stream1 TX, with short SPI end-of-transfer callbacks.
Do not enable other SPI1 sensor clients without implementing bus arbitration.

Each acquisition reads STATUS_REG, then only the sensor outputs whose ready bits
were set. Both-ready samples share a contiguous 12-byte gyro/accelerometer burst.
Block-data-update and address auto-increment stay enabled. Runtime reads never
call polling SPI GetAxes/GetSensitivity functions or wait for completion.
Only boot-time configuration uses bounded synchronous register transfers.

Two 32-byte banks each hold six int32 raw sums and separate accel/gyro counts.
The IRQ producer adds three signed raw values per ready sensor. At each model
update a short interrupt-protected bank swap hands the completed bank to the
main loop. All conversion and division occur outside interrupts, once per batch,
with one reciprocal/scale per sensor. Accelerometer means are m/s^2; gyro means
are deg/s, converted to rad/s by the existing model wrapper. SD lsm_accel/lsm_gyro
fields now contain these same means rather than a single register snapshot.
There is no per-sample float math, ring traversal, allocation, or sample copy history.

A normal 10 ms window contains about 16-17 samples per sensor. Longer actual
intervals (for example a missed timer deadline) include more completed samples; counts can
differ between sensors, and each uses its own divisor. An empty sensor window
skips the model/log sample instead of passing stale data; existing sums are kept
until both sensors have samples. The LOCKOUT close deadline still runs even if
IMU sampling fails. A DMA already in flight at bank swap contributes to the next
window when it completes. Startup mount/header samples are discarded before
the first scheduled model window.

Sums stop accepting samples at 65535 per sensor per window to prevent int32
overflow. `lsm_last_accel_samples` / `lsm_last_gyro_samples` expose the last batch
counts. `lsm_accel_sample_count`, `lsm_gyro_sample_count`, `lsm_drdy_count`,
`lsm_drdy_busy_count`, `lsm_spi_error_count`, `lsm_empty_window_count`, and
`lsm_accumulator_overflow_count` are debugger-visible diagnostics. A busy DRDY
queues one status recheck; it is not itself proof of a lost sample. Hardware
registers are not a FIFO, so extended interrupt masking can still lose samples.
SPI errors disable acquisition rather than run a blocking abort/reinit in an IRQ.
Aligned, separate 32-byte DMA buffers live in DMA-accessible AXI SRAM; explicit
cache maintenance covers configurations with the CM7 D-cache enabled.

The accelerometer is configured for ±8 g with the datasheet sensitivity of
0.244 mg/LSB. The SD sample count fields record the actual number of raw
accelerometer and gyroscope samples used for each logged average.
