import base64
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from navgraph_replay import (
    BroadcastEphemeris,
    decode_subframe,
    evaluate_broadcast_state,
    preprocess_ndjson,
    replay,
)


WEEK_MODULO = 123
ABSOLUTE_WEEK = 2_171
WEEK_SECONDS = 604_800.0
TOE_SECONDS = 320_000.0
IODE = 0x55
IODC = 0x155


class NavigationWordWriter:
    def __init__(self, subframe_id: int) -> None:
        self.data = bytearray(40)
        self.unsigned(49, 3, subframe_id)

    def unsigned(self, index: int, length: int, value: int) -> None:
        if value < 0 or value >= 1 << length:
            raise ValueError("unsigned value outside bit field")
        for offset in range(length):
            bit = (value >> (length - 1 - offset)) & 1
            logical = index + offset
            word = logical // 30
            physical = logical + (word + 1) * 2
            self.data[physical // 8] |= bit << (7 - physical % 8)

    def signed(self, index: int, length: int, value: int) -> None:
        self.unsigned(index, length, value & ((1 << length) - 1))

    def split32(self, index_8: int, index_24: int, value: int) -> None:
        raw = value & 0xFFFFFFFF
        self.unsigned(index_8, 8, raw >> 24)
        self.unsigned(index_24, 24, raw & 0xFFFFFF)


def subframe_1() -> bytes:
    writer = NavigationWordWriter(1)
    writer.unsigned(60, 10, WEEK_MODULO)
    writer.unsigned(72, 4, 2)
    writer.unsigned(76, 6, 0)
    writer.unsigned(82, 2, IODC >> 8)
    writer.signed(196, 8, -10)
    writer.unsigned(210, 8, IODC & 0xFF)
    writer.unsigned(218, 16, int(TOE_SECONDS / 16.0))
    writer.signed(240, 8, -2)
    writer.signed(248, 16, -100)
    writer.signed(270, 22, -50_000)
    return bytes(writer.data)


def subframe_2() -> bytes:
    writer = NavigationWordWriter(2)
    writer.unsigned(60, 8, IODE)
    writer.signed(68, 16, -1_000)
    writer.signed(90, 16, 100)
    writer.split32(106, 120, 0x20000000)
    writer.signed(150, 16, -200)
    writer.split32(166, 180, round(0.01 * math.ldexp(1.0, 33)))
    writer.signed(210, 16, 300)
    writer.split32(226, 240, round(5_153.7954775 * math.ldexp(1.0, 19)))
    writer.unsigned(270, 16, int(TOE_SECONDS / 16.0))
    return bytes(writer.data)


def subframe_3() -> bytes:
    writer = NavigationWordWriter(3)
    writer.signed(60, 16, 120)
    writer.split32(76, 90, -0x30000000)
    writer.signed(120, 16, -80)
    writer.split32(136, 150, 0x40000000)
    writer.signed(180, 16, 2_000)
    writer.split32(196, 210, 0x10000000)
    writer.signed(240, 24, -22_000)
    writer.unsigned(270, 8, IODE)
    writer.signed(278, 14, 500)
    return bytes(writer.data)


def envelope(record_type: str, elapsed_ns: int, **payload):
    return {
        "schema_version": 1,
        "record_type": record_type,
        "session_id": "navgraph_test",
        "elapsed_realtime_nanos": elapsed_ns,
        "wall_time_millis": 1_700_000_000_000,
        **payload,
    }


def nav_record(svid: int, subframe_id: int, elapsed_ns: int):
    payload = (subframe_1(), subframe_2(), subframe_3())[subframe_id - 1]
    return envelope(
        "gnss_navigation_message", elapsed_ns,
        callback_elapsed_realtime_nanos=elapsed_ns,
        svid=svid,
        message_type=257,
        message_id=-1,
        submessage_id=subframe_id,
        status=1,
        data_base64=base64.b64encode(payload).decode("ascii"),
    )


def clock_and_raw(event_ns: int, clock_arc: int, offset_seconds: float = 0.0):
    receive_gps_ns = round((ABSOLUTE_WEEK * WEEK_SECONDS + TOE_SECONDS + 900.0 + offset_seconds) * 1e9)
    time_nanos = 10_000_000_000 + round(offset_seconds * 1e9)
    full_bias_nanos = time_nanos - receive_gps_ns
    receive_tow_ns = round((TOE_SECONDS + 900.0 + offset_seconds) * 1e9)
    transmit_tow_ns = receive_tow_ns - 70_000_000
    clock = envelope(
        "gnss_clock", event_ns,
        time_nanos=time_nanos,
        full_bias_nanos=full_bias_nanos,
        bias_nanos=0.0,
        bias_uncertainty_nanos=3.0,
        hardware_clock_discontinuity_count=clock_arc,
        callback_elapsed_realtime_nanos=event_ns + 1_000,
    )
    measurements = [
        envelope(
            "gnss_raw", event_ns,
            gnss_event_elapsed_realtime_nanos=event_ns,
            callback_elapsed_realtime_nanos=event_ns + 1_000,
            constellation_type=1,
            svid=svid,
            state=9,
            time_offset_nanos=0.0,
            received_sv_time_nanos=transmit_tow_ns,
            received_sv_time_uncertainty_nanos=10,
            cn0_dbhz=35.0,
            pseudorange_rate_mps=0.0,
            pseudorange_rate_uncertainty_mps=0.5,
            accumulated_delta_range_state=1,
            accumulated_delta_range_m=0.0,
            accumulated_delta_range_uncertainty_m=0.1,
            multipath_indicator=0,
            carrier_frequency_hz=1_575_420_000.0,
            code_type="C",
        )
        for svid in range(1, 5)
    ]
    return clock, measurements


def write_log(path: Path, records) -> None:
    path.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")


class BroadcastNavigationTests(unittest.TestCase):
    def test_android_word_layout_and_signed_scales(self) -> None:
        one = decode_subframe(1, subframe_1())
        two = decode_subframe(2, subframe_2())
        three = decode_subframe(3, subframe_3())
        self.assertEqual(WEEK_MODULO, one["week"])
        self.assertEqual(IODC, one["iodc"])
        self.assertEqual(IODE, two["iode"])
        self.assertEqual(IODE, three["iode"])
        self.assertAlmostEqual(-10.0 * math.ldexp(1.0, -31), one["tgd"], places=20)
        self.assertAlmostEqual(-1000.0 * math.ldexp(1.0, -5), two["crs"], places=12)
        self.assertAlmostEqual(0.25 * math.pi, two["m0"], places=10)
        self.assertAlmostEqual(-0.375 * math.pi, three["omega0"], places=10)

    def test_broadcast_state_matches_android_regression(self) -> None:
        ephemeris = BroadcastEphemeris(
            svid=7, week_modulo_1024=WEEK_MODULO, ura_index=2, sv_health=0,
            iodc=IODC, iode=IODE,
            tgd_s=-10.0 * math.ldexp(1.0, -31), toc_s=TOE_SECONDS,
            af0_s=-50_000.0 * math.ldexp(1.0, -31),
            af1_sps=-100.0 * math.ldexp(1.0, -43),
            af2_sps2=-2.0 * math.ldexp(1.0, -55),
            crs_m=-1_000.0 * math.ldexp(1.0, -5),
            delta_n_radps=100.0 * math.ldexp(1.0, -43) * math.pi,
            m0_rad=0.25 * math.pi,
            cuc_rad=-200.0 * math.ldexp(1.0, -29), eccentricity=0.01,
            cus_rad=300.0 * math.ldexp(1.0, -29), sqrt_a_sqrt_m=5_153.7954775,
            toe_s=TOE_SECONDS, cic_rad=120.0 * math.ldexp(1.0, -29),
            omega0_rad=-0.375 * math.pi, cis_rad=-80.0 * math.ldexp(1.0, -29),
            i0_rad=0.5 * math.pi, crc_m=2_000.0 * math.ldexp(1.0, -5),
            omega_rad=0.125 * math.pi,
            omega_dot_radps=-22_000.0 * math.ldexp(1.0, -43) * math.pi,
            i_dot_radps=500.0 * math.ldexp(1.0, -43) * math.pi,
        )
        query_ns = round((ABSOLUTE_WEEK * WEEK_SECONDS + TOE_SECONDS + 900.0) * 1e9)
        state = evaluate_broadcast_state(ephemeris, query_ns)
        self.assertIsNotNone(state)
        np.testing.assert_allclose(
            state.position_m,
            [5_454_952.158701, 3_376_364.600164, 25_610_318.325144],
            atol=1e-3,
        )
        np.testing.assert_allclose(
            state.velocity_mps,
            [-2_961.726239, -2_383.408329, 977.138807],
            atol=1e-3,
        )
        self.assertAlmostEqual(-6_987.252586, state.clock_bias_m, places=6)
        self.assertAlmostEqual(-0.004044657, state.clock_drift_mps, places=9)


class NdjsonPreprocessingTests(unittest.TestCase):
    def test_reconstructs_epochs_across_clock_discontinuity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "device.ndjson"
            records = [envelope("session_start", 1, app_version="0.7.0", requested_imu_period_us=10_000)]
            for svid in range(1, 5):
                for subframe in range(1, 4):
                    records.append(nav_record(svid, subframe, 1_000_000 + svid * 10_000 + subframe))
            clock_one, raw_one = clock_and_raw(10_000_000_000, 0)
            clock_two, raw_two = clock_and_raw(11_000_000_000, 1, 1.0)
            # File order is deliberately not stream order; replay must use event time.
            records.extend(raw_one)
            records.append(clock_one)
            records.append(clock_two)
            records.extend(raw_two)
            records.append(envelope("session_end", 12_000_000_000, reason="test"))
            write_log(path, records)

            result = preprocess_ndjson(path)
            self.assertEqual([0, 1], result.clock_arcs)
            self.assertEqual(2, len(result.epochs))
            self.assertEqual(8, sum(len(item.epoch.observations) for item in result.epochs))
            self.assertEqual(4, result.navigation_health.complete_ephemerides)
            self.assertEqual({}, dict(result.rejection_counts))
            summary = replay(path, Path(directory) / "solver_output")
            self.assertEqual("completed", summary["solver_status"])
            self.assertIn("pseudorange", summary["jacobian_max_abs_error"])

    def test_future_navigation_message_is_not_used(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "future_nav.ndjson"
            clock, measurements = clock_and_raw(10_000_000_000, 0)
            records = [envelope("session_start", 1, app_version="0.7.0", requested_imu_period_us=10_000)]
            records.extend(nav_record(svid, subframe, 20_000_000_000 + svid * 10 + subframe)
                           for svid in range(1, 5) for subframe in range(1, 4))
            records.append(clock)
            records.extend(measurements)
            records.append(envelope("session_end", 30_000_000_000, reason="test"))
            write_log(path, records)

            result = preprocess_ndjson(path)
            self.assertEqual(0, len(result.epochs))
            self.assertEqual(4, result.rejection_counts["unavailable_ephemeris"])
            self.assertEqual(1, result.rejection_counts["insufficient_observations"])

    def test_missing_full_bias_rejects_the_complete_callback(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing_bias.ndjson"
            clock, measurements = clock_and_raw(10_000_000_000, 0)
            del clock["full_bias_nanos"]
            records = [
                envelope("session_start", 1, app_version="0.7.0", requested_imu_period_us=10_000),
                clock,
                *measurements,
                envelope("session_end", 11_000_000_000, reason="test"),
            ]
            write_log(path, records)

            result = preprocess_ndjson(path)
            self.assertEqual(0, len(result.epochs))
            self.assertEqual(4, result.rejection_counts["missing_clock_bias"])

    def test_replay_writes_parseable_empty_solver_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "emulator.ndjson"
            clock, measurements = clock_and_raw(10_000_000_000, 0)
            records = [
                envelope("session_start", 1, app_version="0.7.0", requested_imu_period_us=10_000),
                envelope(
                    "imu", 2, sensor_type="ACCELEROMETER", sensor_name="test",
                    sensor_timestamp_nanos=2, callback_elapsed_realtime_nanos=3,
                    accuracy=3, values=[0.0, 0.0, 9.81],
                ),
                envelope(
                    "fused_location", 4, provider="fused", latitude_deg=1.3521,
                    longitude_deg=103.8198, altitude_m=20.0, accuracy_m=5.0,
                    location_time_millis=1_700_000_000_000,
                    location_elapsed_realtime_nanos=4, callback_elapsed_realtime_nanos=5,
                ),
                clock,
                *measurements,
                envelope("session_end", 11_000_000_000, reason="test"),
            ]
            write_log(path, records)
            output = root / "output"

            summary = replay(path, output)
            self.assertEqual("not_run_no_clean_epochs", summary["solver_status"])
            self.assertTrue((output / "summary.json").is_file())
            self.assertTrue((output / "clean_observations.csv").is_file())
            self.assertTrue((output / "solutions.csv").is_file())
            self.assertIn("Android fused", (output / "trajectory.svg").read_text(encoding="utf-8"))
            self.assertEqual(1, summary["imu"]["accelerometer"]["samples"])
            serialized = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual("not_run_no_clean_epochs", serialized["solver_status"])


if __name__ == "__main__":
    unittest.main()
