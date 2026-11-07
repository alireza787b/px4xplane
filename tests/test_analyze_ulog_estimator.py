import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location(
    "analyze_ulog_estimator", Path(__file__).parents[1] / "tools/analyze_ulog_estimator.py")
estimator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(estimator)


class EstimatorCompatibilityTests(unittest.TestCase):
    def test_current_nested_gnss_fields_are_summarized(self):
        dataset = SimpleNamespace(name="vehicle_gnss", multi_id=0, data={
            "timestamp": [1000000, 2000000], "receiver.vel_down": [0.0, 2.0],
            "receiver.altitude_msl": [100.0, 98.0], "usable": [True, True]})
        summary = estimator.topic_summary(dataset)
        self.assertEqual(summary["fields"]["receiver.vel_down"]["max"], 2.0)
        self.assertIn("vehicle_gnss", estimator.TOPICS)
        self.assertIn("sensor_gnss", estimator.TOPICS)

    def analyze_state(self, current):
        old = SimpleNamespace(name="commander_state", data={"timestamp": [1, 2], "main_state": [0, 1]})
        status = SimpleNamespace(name="vehicle_status", data={
            "timestamp": [1, 2], "nav_state": [4, 18], "arming_state": [1, 2]})
        ulog = SimpleNamespace(data_list=[old, status] if current else [old], logged_messages=[])
        with patch.object(estimator, "load_ulog", return_value=ulog):
            return estimator.analyze(Path("fixture.ulg"), 15.0)

    def test_current_status_preferred_for_mode_transitions(self):
        result = self.analyze_state(True)
        self.assertEqual(result["state_topic"], "vehicle_status")
        self.assertEqual({t["field"] for t in result["commander_transitions"]}, {"nav_state", "arming_state"})

    def test_legacy_status_and_gps_fields_still_supported(self):
        result = self.analyze_state(False)
        self.assertEqual(result["state_topic"], "commander_state")
        dataset = SimpleNamespace(name="vehicle_gps_position", data={
            "timestamp": [1, 2], "vel_d_m_s": [1.0, 2.0]})
        self.assertEqual(estimator.topic_summary(dataset)["fields"]["vel_d_m_s"]["max"], 2.0)

    def test_gnss_warning_is_not_silently_omitted(self):
        messages = [{"message": "GNSS receiver unusable"}, {"message": "ordinary status"}]
        self.assertEqual(estimator.relevant_events(messages), messages[:1])


if __name__ == "__main__":
    unittest.main()
