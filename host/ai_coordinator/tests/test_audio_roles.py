from pathlib import Path
import tempfile
import unittest
from ai_coordinator.device_roles import audio_device,audio_inventory

class AudioRoleTests(unittest.TestCase):
    def test_card_numbers_and_names_can_change_without_switching_usb_identity(self):
        selector={'vid':0x0d8c,'pid':0x12}
        inventory=[dict(role='speaker',vid=0x0d8c,pid=0x12,device='plughw:CARD=Device_1,DEV=0',aliases=[]),
                   dict(role='speaker',vid=1,pid=2,device='plughw:CARD=Device,DEV=0',aliases=[])]
        self.assertEqual(audio_device(selector,'speaker',inventory=inventory),'plughw:CARD=Device_1,DEV=0')
        with self.assertRaises(ValueError):audio_device(selector,'speaker',inventory=inventory[:1]*2)
        with self.assertRaises(ValueError):audio_device(selector,'microphone',inventory=inventory)
    def test_missing_identity_never_falls_back_to_hdmi_or_first_card(self):
        with self.assertRaises(ValueError):audio_device({'vid':1,'pid':2},'speaker',legacy='plughw:CARD=HDMI,DEV=0',inventory=[])
        self.assertEqual(audio_device(None,'speaker','plughw:CARD=Old,DEV=0'),'plughw:CARD=Old,DEV=0')
    def test_inventory_uses_usb_ancestry_and_pcm_direction(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);sound=root/'sys/class/sound';device=sound/'card9/device';device.mkdir(parents=True)
            (device/'idVendor').write_text('4c4a');(device/'idProduct').write_text('4c55');(device/'serial').write_text('my-mic')
            (sound/'pcmC9D0c').mkdir();(sound/'pcmC9D1p').mkdir()
            proc=root/'proc/asound/card9';proc.mkdir(parents=True);(proc/'id').write_text('USB_1')
            items=audio_inventory(root/'sys',root/'proc')
            self.assertEqual({x['role'] for x in items},{'microphone','speaker'})
            self.assertTrue(all(x['serial']=='my-mic' and 'USB_1' in x['device'] for x in items))

if __name__=='__main__':unittest.main()
