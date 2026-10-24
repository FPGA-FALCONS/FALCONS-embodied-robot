import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.doctor import inspect_config


class DoctorTests(unittest.TestCase):
    def test_deep_check_detects_same_size_corruption_and_never_claims_hardware(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);model=root/'weights';model.write_bytes(b'1234')
            cfg=root/'config.json';cfg.write_text(json.dumps({'mode':'shadow','model_loads_enabled':False,
                'artifacts':{'test':{'path':str(model),'bytes':4,'sha256':hashlib.sha256(b'1234').hexdigest()}}}))
            result=inspect_config(cfg,True);self.assertTrue(result['artifacts_ok'])
            self.assertFalse(result['configured_for_live']);self.assertFalse(result['physical_acceptance_verified'])
            model.write_bytes(b'4321');self.assertFalse(inspect_config(cfg,True)['artifacts_ok'])
