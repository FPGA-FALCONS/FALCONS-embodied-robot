import copy
import unittest
from ai_coordinator.inspection.usb_link import select


class USBSelectionTests(unittest.TestCase):
    def setUp(self):
        self.cfg={'selector':{'vid':0x04b4,'pid':0x00f1}}
        self.device={'path':'5-1','vid':'04b4','pid':'00f1','speed':'480','busnum':'5','devnum':'6','serial':'',
                     'interfaces':[{'bInterfaceNumber':'00','bAlternateSetting':'00','driver':None,'endpoints':[
                         {'bEndpointAddress':'02','bmAttributes':'02','wMaxPacketSize':'0200'},
                         {'bEndpointAddress':'86','bmAttributes':'02','wMaxPacketSize':'0200'}]}]}

    def test_unique_identity_can_move_to_another_socket(self):
        before=select(self.cfg,[self.device]);self.device.update(path='7-3',busnum='7',devnum='9')
        after=select(self.cfg,[self.device]);self.assertEqual(before['pid'],after['pid'])
        self.assertEqual((after['bus_number'],after['device_address']),(7,9))

    def test_ambiguity_driver_speed_and_endpoint_contract_are_rejected(self):
        with self.assertRaises(ValueError):select(self.cfg,[self.device,self.device])
        for mutate in ('speed','driver','packet','endpoint'):
            d=copy.deepcopy(self.device)
            if mutate=='speed':d['speed']='12'
            elif mutate=='driver':d['interfaces'][0]['driver']='other'
            elif mutate=='packet':d['interfaces'][0]['endpoints'][0]['wMaxPacketSize']='0040'
            else:d['interfaces'][0]['endpoints'].pop()
            with self.subTest(mutate=mutate),self.assertRaises(ValueError):select(self.cfg,[d])
