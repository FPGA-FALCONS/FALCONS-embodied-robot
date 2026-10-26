import copy
from pathlib import Path
import unittest
from ai_coordinator.deploy_inspection import prepare_config

class DeploymentTests(unittest.TestCase):
    def test_preserves_existing_models_audio_and_motion_configuration(self):
        current=dict(root='/home/orangepi/deploy',models={'chat':{'provider':'rkllm','path':'existing'},'asr':{'model':'existing-asr'}},
                     ai_module={'speaker_device':'plughw:CARD=Device,DEV=0'},microphone={'device':'plughw:CARD=Mic,DEV=0'},
                     gateway={'mode':'serial','allow_motion':False},motion_backend={'speed':30},camera={'enabled':False})
        before=copy.deepcopy(current)
        defaults={'enabled':False,'camera':{'selector':{'vid':1,'pid':2,'serial':'camera'},'expected_size':[1280,720]},
                  'calibration':{'intrinsics_measured':False},'speed_mapping_verified':False}
        cfg=prepare_config(current,defaults,Path('/prepared/release'))
        self.assertEqual(current,before)
        for key in ('models','ai_module','microphone','gateway','motion_backend'):self.assertEqual(cfg[key],before[key])
        self.assertTrue(cfg['inspection']['enabled']);self.assertTrue(cfg['camera']['enabled'])
        self.assertEqual(cfg['camera']['backend'],'v4l2_native')
        self.assertFalse(cfg['inspection']['calibration']['intrinsics_measured'])
        self.assertNotIn('device',cfg['camera'])
    def test_previously_measured_profile_is_retained_when_updating_source(self):
        cfg=prepare_config({'root':'/deploy','inspection':{'calibration':{'measured_marker':'retain'},'camera':{'selector':{'serial':'my-camera'}}}},
                           {'camera':{},'calibration':{'intrinsics_measured':False}},Path('/new'))
        self.assertEqual(cfg['inspection']['calibration'],{'measured_marker':'retain'})
        self.assertEqual(cfg['camera']['selector']['serial'],'my-camera')

if __name__=='__main__':unittest.main()
