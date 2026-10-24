from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'voice_bridge'))
from domain_vocab import command_text


class DomainVocabTests(unittest.TestCase):
    def test_search_homophone_preserves_raw(self):
        result=command_text('先找交代，找不到再找螺丝刀，找到后返回起点')
        self.assertIn('交代',result['raw_text']);self.assertNotIn('交代',result['task_text'])
        self.assertEqual(result['task_text'],'先找胶带，找不到再找螺丝刀，找到后返回起点')

    def test_negation_is_preserved(self):
        self.assertEqual(command_text('不要找交代')['task_text'],'不要找胶带')
        self.assertEqual(command_text('找到交代以后返回起点')['task_text'],'找到胶带以后返回起点')

    def test_general_language_is_not_a_search(self):
        for text in ('请把事情交代清楚','交代一下任务','我已经交代过了','交代'):
            with self.subTest(text=text):
                result=command_text(text)
                self.assertTrue(result['blocked']);self.assertIsNone(result['task_text'])

    def test_unresolved_second_occurrence_blocks(self):
        self.assertTrue(command_text('找交代，然后交代一下结果')['blocked'])

    def test_unrelated_command_is_unchanged(self):
        for text in ('立即停止','找营养快线','不要停止',''):
            self.assertEqual(command_text(text)['task_text'],text)


if __name__=='__main__':unittest.main()
