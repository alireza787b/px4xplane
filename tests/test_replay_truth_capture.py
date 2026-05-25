#!/usr/bin/env python3

import csv
import tempfile
import unittest
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import replay_truth_capture as replay


class ReplayTruthCaptureTest(unittest.TestCase):
    def test_decodes_sensor_and_gps_contract(self):
        row = {
            "frame_id": "7",
            "sim/time/total_flight_time_sec": "12.5",
            "sim/flightmodel/forces/g_axil": "0.1",
            "sim/flightmodel/forces/g_side": "0.2",
            "sim/flightmodel/forces/g_nrml": "1.0",
            "sim/flightmodel/position/Prad": "0.01",
            "sim/flightmodel/position/Qrad": "0.02",
            "sim/flightmodel/position/Rrad": "0.03",
            "sim/flightmodel/position/indicated_airspeed": "10",
            "sim/flightmodel/position/true_airspeed": "12",
            "sim/flightmodel/position/elevation": "100",
            "sim/cockpit2/temperature/outside_air_temp_degc": "15",
            "sim/flightmodel/position/latitude": "26.5",
            "sim/flightmodel/position/longitude": "54.0",
            "sim/flightmodel/position/local_vx": "3",
            "sim/flightmodel/position/local_vy": "-1",
            "sim/flightmodel/position/local_vz": "-4",
            "sim/flightmodel/position/psi": "90",
            "sim/flightmodel/position/q": "1;0;0;0",
        }

        sensor = replay.decode_sensor(row, 1_000_000)
        self.assertAlmostEqual(sensor["xacc"], -0.980665)
        self.assertAlmostEqual(sensor["yacc"], 1.96133)
        self.assertAlmostEqual(sensor["zacc"], -9.80665)
        self.assertGreater(sensor["diff_pressure_hpa"], 0.0)
        self.assertEqual(sensor["pressure_alt"], 0.0)

        gps, cog = replay.decode_gps(row, 1_000_000, 0)
        self.assertEqual(gps["vn_cms"], 400)
        self.assertEqual(gps["ve_cms"], 300)
        self.assertEqual(gps["vd_cms"], 100)
        self.assertEqual(gps["vel_cms"], 500)
        self.assertEqual(gps["yaw_cdeg"], 9000)
        self.assertEqual(gps["gps_id"], 0)
        self.assertEqual(cog, gps["cog_cdeg"])

        state = replay.decode_state(row, 1_000_000)
        self.assertEqual(state["xacc"], -100)
        self.assertEqual(state["yacc"], 200)
        self.assertEqual(state["zacc"], -1000)
        self.assertEqual(state["ind_airspeed_cms"], 514)

    def test_streams_capture_folder(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir)
            fields = [
                "frame_id",
                "sim/time/total_flight_time_sec",
                "sim/flightmodel/forces/g_axil",
                "sim/flightmodel/forces/g_side",
                "sim/flightmodel/forces/g_nrml",
                "sim/flightmodel/position/elevation",
                "sim/flightmodel/position/local_vx",
                "sim/flightmodel/position/local_vy",
                "sim/flightmodel/position/local_vz",
            ]
            with (path / "frames.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow(
                    {
                        "frame_id": "1",
                        "sim/time/total_flight_time_sec": "1.0",
                        "sim/flightmodel/forces/g_axil": "0",
                        "sim/flightmodel/forces/g_side": "0",
                        "sim/flightmodel/forces/g_nrml": "1",
                        "sim/flightmodel/position/elevation": "10",
                        "sim/flightmodel/position/local_vx": "0",
                        "sim/flightmodel/position/local_vy": "0",
                        "sim/flightmodel/position/local_vz": "0",
                    }
                )

            args = type(
                "Args",
                (),
                {
                    "capture": str(path),
                    "messages": "sensor",
                    "sensor_rate_hz": 200.0,
                    "gps_rate_hz": 20.0,
                    "state_rate_hz": 10.0,
                    "rc_rate_hz": 10.0,
                    "max_frames": 0,
                    "max_messages": 0,
                },
            )()
            rows = list(replay.iter_decoded_rows(args))

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["message"], "HIL_SENSOR")

    def test_preserves_signed_differential_pressure(self):
        row = {
            "frame_id": "1",
            "sim/time/total_flight_time_sec": "1.0",
            "sim/flightmodel/position/indicated_airspeed": "-20",
            "sim/flightmodel/position/elevation": "0",
        }

        sensor = replay.decode_sensor(row, 1_000_000)
        self.assertLess(sensor["diff_pressure_hpa"], 0.0)
        self.assertAlmostEqual(
            abs(sensor["diff_pressure_hpa"]),
            replay.signed_dynamic_pressure_hpa_from_ias_knots(20),
        )

    def test_stationary_ground_contract_masks_contact_spikes(self):
        row = {
            "frame_id": "2",
            "sim/time/total_flight_time_sec": "2.0",
            "sim/flightmodel/failures/onground_any": "1",
            "sim/flightmodel/position/y_agl": "0.1",
            "sim/flightmodel/forces/g_axil": "3.5",
            "sim/flightmodel/forces/g_side": "-2.0",
            "sim/flightmodel/forces/g_nrml": "0.2",
            "sim/flightmodel/position/Prad": "1.0",
            "sim/flightmodel/position/Qrad": "-2.0",
            "sim/flightmodel/position/Rrad": "3.0",
            "sim/flightmodel/position/indicated_airspeed": "0",
            "sim/flightmodel/position/elevation": "52.5",
            "sim/flightmodel/position/latitude": "26.5",
            "sim/flightmodel/position/longitude": "54.0",
            "sim/flightmodel/position/local_vx": "0.05",
            "sim/flightmodel/position/local_vy": "0.0",
            "sim/flightmodel/position/local_vz": "0.0",
            "sim/flightmodel/position/psi": "90",
            "sim/flightmodel/position/q": "1;0;0;0",
        }
        contract = replay.GroundStationaryContract()
        contract.update(row)

        sensor = replay.decode_sensor(row, 1_000_000, contract)
        self.assertAlmostEqual(sensor["xacc"], 0.0)
        self.assertAlmostEqual(sensor["yacc"], 0.0)
        self.assertAlmostEqual(sensor["zacc"], -replay.GRAVITY_M_S2)
        self.assertEqual(sensor["xgyro"], 0.0)
        self.assertEqual(sensor["ygyro"], 0.0)
        self.assertEqual(sensor["zgyro"], 0.0)

        gps, _ = replay.decode_gps(row, 1_000_000, 0, contract)
        self.assertEqual(gps["vn_cms"], 0)
        self.assertEqual(gps["ve_cms"], 0)
        self.assertEqual(gps["vd_cms"], 0)
        self.assertEqual(gps["alt_mm"], 52500)

        state = replay.decode_state(row, 1_000_000, contract)
        self.assertEqual(state["rollspeed"], 0.0)
        self.assertEqual(state["pitchspeed"], 0.0)
        self.assertEqual(state["yawspeed"], 0.0)
        self.assertEqual(state["xacc"], 0)
        self.assertEqual(state["yacc"], 0)
        self.assertEqual(state["zacc"], -1000)


if __name__ == "__main__":
    unittest.main()
