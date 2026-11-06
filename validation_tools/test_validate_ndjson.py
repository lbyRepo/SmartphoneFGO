import base64
import json
import tempfile
import unittest
from pathlib import Path

from validate_ndjson import validate


class ValidateNdjsonNavigationMessageTest(unittest.TestCase):
    def test_accepts_android_gps_l1_ca_payload(self):
        result = self._validate(base64.b64encode(bytes(40)).decode("ascii"))
        self.assertTrue(result["valid"], result["errors"])
        self.assertEqual(1, result["record_counts"]["gnss_navigation_message"])

    def test_rejects_wrong_gps_l1_ca_payload_length(self):
        result = self._validate(base64.b64encode(bytes(39)).decode("ascii"))
        self.assertFalse(result["valid"])
        self.assertIn("GPS L1 C/A payload must be 40 bytes", "\n".join(result["errors"]))

    def _validate(self, encoded_payload):
        session = "test_session"
        records = [
            self._record("session_start", 1, session, app_version="0.7.0", requested_imu_period_us=10_000),
            self._record(
                "gnss_navigation_message",
                2,
                session,
                callback_elapsed_realtime_nanos=2,
                svid=7,
                message_type=257,
                message_id=-1,
                submessage_id=1,
                status=1,
                data_base64=encoded_payload,
            ),
            self._record("session_end", 3, session, reason="test"),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.ndjson"
            path.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")
            return validate(path, require_all_streams=False)

    @staticmethod
    def _record(record_type, event_time, session_id, **payload):
        return {
            "schema_version": 1,
            "record_type": record_type,
            "session_id": session_id,
            "elapsed_realtime_nanos": event_time,
            "wall_time_millis": event_time,
            **payload,
        }


if __name__ == "__main__":
    unittest.main()
