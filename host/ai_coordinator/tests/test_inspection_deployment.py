import copy
from pathlib import Path
import unittest
from ai_coordinator.deploy_inspection import prepare_config,apply_ground_homography

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
    def test_focus_defaults_and_existing_measured_focus_are_preserved(self):
        defaults={'camera':{'focus':260,'autofocus':False}}
        for old,expected in (({},260),({'focus':285,'autofocus':False},285)):
            cfg=prepare_config({'root':'/deploy','camera':old,'inspection':{'camera':{'expected_size':[1280,720]}}},defaults,Path('/new'))
            self.assertEqual(cfg['camera']['focus'],expected)
            self.assertIs(cfg['camera']['autofocus'],False)
            self.assertEqual(cfg['inspection']['camera'],cfg['camera'])

    def test_previously_measured_profile_is_retained_when_updating_source(self):
        cfg=prepare_config({'root':'/deploy','inspection':{'calibration':{'measured_marker':'retain'},'camera':{'selector':{'serial':'my-camera'}}}},
                           {'camera':{},'calibration':{'intrinsics_measured':False}},Path('/new'))
        self.assertEqual(cfg['inspection']['calibration'],{'measured_marker':'retain'})
        self.assertEqual(cfg['camera']['selector']['serial'],'my-camera')

    def test_explicit_homography_only_changes_calibration_not_motion_or_models(self):
        import json
        profile=json.loads((Path(__file__).parents[1]/'profiles/inspection_ground_homography_20261028.json').read_text(encoding='utf-8'))
        cfg={'camera':{'selector':{'serial':'200901010001'},'expected_size':[1280,720],
                       'focus':260,'autofocus':False},'inspection':{'calibration':{'old':True},
                       'capture_timing':{'measured':False},'speed_mapping_verified':False,
                       'ground_anchor_policies':{},'vehicle_braking_verified':False},
             'models':{'chat':{'model':'original'}},'ai_module':{'enabled':True}}
        before=copy.deepcopy(cfg);changed=apply_ground_homography(cfg,profile)
        self.assertEqual(cfg,before)
        self.assertEqual(changed['inspection']['calibration'],profile)
        changed['inspection']['calibration']=before['inspection']['calibration']
        self.assertEqual(changed,before)

    def test_explicit_homography_rejects_camera_binding_or_raw_model_mismatch(self):
        import json
        profile=json.loads((Path(__file__).parents[1]/'profiles/inspection_ground_homography_20261028.json').read_text(encoding='utf-8'))
        camera={'selector':{'serial':'200901010001'},'expected_size':[1280,720],'focus':260,'autofocus':False}
        for change in ({'focus':285},{'autofocus':True},{'expected_size':[1920,1080]},
                       {'selector':{'serial':'other'}}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                apply_ground_homography({'camera':{**camera,**change},'inspection':{}},profile)
        wrong=copy.deepcopy(profile);wrong['homography']['pixel_domain']='raw_pixels'
        with self.assertRaises(ValueError):apply_ground_homography({'camera':camera,'inspection':{}},wrong)

if __name__=='__main__':unittest.main()
