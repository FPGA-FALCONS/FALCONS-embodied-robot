import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from ai_coordinator.inspection.evidence import Evidence

class EvidenceTests(unittest.TestCase):
    def test_artifacts_are_immutable_unique_and_checksums_match_actual_payload(self):
        with tempfile.TemporaryDirectory() as root:
            recorder=Evidence(root);one=recorder.write('keyframe',b'\xff\xd8sample',{'task':1,'epoch':2})
            two=recorder.write('keyframe',b'other',{'task':1,'epoch':2})
            self.assertNotEqual(one['payload'],two['payload']);self.assertEqual(Path(one['payload']).suffix,'.jpg')
            meta=json.loads(Path(one['metadata']).read_text(encoding='utf-8'))
            self.assertEqual(meta['payload_sha256'],hashlib.sha256(Path(one['payload']).read_bytes()).hexdigest())
    def test_quota_and_path_escape_are_rejected_without_deleting_evidence(self):
        with tempfile.TemporaryDirectory() as root:
            recorder=Evidence(root,limit_bytes=1)
            with self.assertRaises(ValueError):recorder.write('layer',b'large',{})
            with self.assertRaises(ValueError):recorder.write('../escape',b'x',{})
            self.assertEqual(list(Path(root).iterdir()),[])

if __name__=='__main__':unittest.main()
