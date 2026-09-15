# Recorded reference and GNSS measurement roles

Date: September 15, 2026. Scope: Mississippi drive
`raw_data_2024-06-07-12-09-31_0.bag`, existing GNSS CSV exports, and the
current replay exporter. This is a data-provenance and experiment-design
finding; no map, observer, gain, input channel, or historical score is changed.

## Recommended interpretation

Use native INSPVA, accompanied by INSPVAX/INSSTDEV quality information, as a
provisional same-receiver navigation reference. This avoids the already
documented ODOM position-source substitutions; it does not establish that
INSPVA is always more accurate or is independent ground truth. A claim of
centimetre-level physical accuracy requires a validated reference with known
uncertainty, common timestamps, coordinate datum, and vehicle reference point.
INS_SOLUTION_GOOD alone is not that validation. Retain the full trajectory
and report any quality-stratified scores with coverage; do not silently drop
poor-reference intervals from headline metrics.

BESTGNSSPOS is the appropriate receiver position log when the intended input
is GNSS without INS. Its output point is the antenna phase centre, so its
lever arm must be transformed consistently before use by a body-position
observer. It is absent as a named topic and as message ID 1429 in all 17,586
recorded `/novatel/oem7/oem7raw` messages in this bag. This does not establish
whether separate receiver files or other recordings contain it, or whether
a different GNSS solution could be reconstructed. No reconstruction is attempted.

BESTPOS must be interpreted from its position type. All 1,170 exported
samples in this drive are INS-assisted positions. They can be described as
external GNSS/INS position aiding, but not as a GNSS-only baseline. ODOM is a
ROS container, not an accuracy class or a separate sensor.

Even an INS-free GNSS position from the same receiver would share satellite
information with an INSPVA evaluation reference. A comparison would measure
agreement with that reference, not independent absolute accuracy. Fusion
should account for shared information instead of assuming independence.

## Observed quality

| Log | Field | Value | Samples |
|---|---|---|---:|
| INSPVA | ins_status | 3: INS_SOLUTION_GOOD | 5,647 |
| INSPVA | ins_status | 6: INS_SOLUTION_FREE | 200 |
| INSPVAX | ins_status | 3 / 6 | 113 / 4 |
| BESTPOS | solution_status | 0: SOL_COMPUTED | 1,170 |
| BESTPOS | position_type | 56: INS_RTKFIXED | 437 |
| BESTPOS | position_type | 55: INS_RTKFLOAT | 683 |
| BESTPOS | position_type | 54: INS_PSRDIFF | 50 |

Status 6 means that the INS is navigating but suspects GNSS error and is
not accepting available aiding updates. It does not by itself prove the
reported position is wrong. BESTPOS latitude/longitude reported standard
deviation medians are 0.1706551/0.1567725 m. These are receiver uncertainty
estimates, not measured errors. The record does not support treating every
sample as independently validated centimetre-level truth.

## Current implementation and map limitation

`scripts/prepareMncavObserverReplay.m` lines 49-54 builds both its reference
and `sensorData.gps` from ODOM XY. Its `gpsInput` metadata already identifies
ODOM, but the header's "GNSS XY" wording is less precise. A future input
revision must explicitly identify GNSS/INS aiding or use an actual GNSS-only
source. This audit does not silently change an existing experiment.

The existing map also uses recorded ODOM poses; its source substitutions were
established in `research/mncav_error_diagnosis_20260914/reference_audit.json`.
Merely changing an evaluation reference does not repair that map. INSPVA
reference consistency, mapping-pose consistency, and absolute physical
accuracy are distinct questions.

## Reproduction and limits

Run `audit.py` using its documented command. It hashes the three input CSVs,
counts all status values, inventories every bag connection, deserializes
every Oem7RawMsg, and counts the little-endian message ID at byte offset 4
after checking both standard and short-header synchronization bytes. Counts
must reconcile with bag connection metadata. It does not validate payload
CRC, estimate localization accuracy, or repeat a matching/observer experiment.
All 17,586 headers were recognized: 14,618 short and 2,968 standard. No raw
message ID was 1429. CSV and raw topics need not have identical counts:
BESTPOS has 1,170 decoded-topic samples and 1,169 raw-topic messages.

## Primary definitions consulted

- [INSPVA: native INS position, velocity and attitude](https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVA.htm).
- [BESTPOS: SPAN behavior and position-type codes](https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm).
- [BESTGNSSPOS: INS-free position, antenna phase centre, ID 1429](https://docs.novatel.com/OEM7/Content/SPAN_Logs/BESTGNSSPOS.htm).
- [INS status definitions](https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSATT.htm).
- [Standard binary header](https://docs.novatel.com/OEM7/Content/Messages/Binary.htm) and
  [short headers](https://docs.novatel.com/OEM7/Content/Messages/Description_of_Short_Headers.htm).

The current official documentation was consulted for message semantics;
the deployed 2024 driver revision and receiver configuration remain unverified.
