import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from semantic_nav import send_motion


class SenderTests(unittest.TestCase):
    def test_distance_negation_compound_and_bad_options_refused(self):
        for text in ('前进两米','不要前进','前进然后左转','左转九十点五度'):
            with self.subTest(text=text),self.assertRaises(ValueError):send_motion.prepare_frame(text)
        for seq,speed in ((-1,40),(256,40),(1,-1),(1,256),(1,41)):
            with self.assertRaises(ValueError):send_motion.prepare_frame('前进',seq,speed)

    def test_preview_never_opens_serial(self):
        with patch.object(send_motion,'open_serial') as opened,patch('sys.stdout',new=io.StringIO()):
            self.assertEqual(send_motion.main(['--dry-run','原地逆时针转一圈']),0)
            self.assertEqual(send_motion.main(['--dry-run','前进两米']),2)
            opened.assert_not_called()

    def test_partial_writes_keep_entire_frame(self):
        _,frame=send_motion.prepare_frame('前进');expected=frame.encode();seen=[]
        def write(fd,data):
            count=min(3,len(data));seen.append(bytes(data[:count]));return count
        with patch.object(send_motion.os,'write',side_effect=write):send_motion.write_frame(123,expected)
        self.assertEqual(b''.join(seen),expected)


if __name__=='__main__':unittest.main()
