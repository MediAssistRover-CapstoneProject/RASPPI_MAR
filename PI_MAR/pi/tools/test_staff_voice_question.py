"""Staff directory questions must reach the BE instead of document search."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.voice_chat import staff_question_candidate


class StaffVoiceQuestionTests(unittest.TestCase):
    def test_named_employee_existence_questions_use_backend(self):
        for question in (
            'Có nhân viên nào tên Trung không?',
            'Có nhân viên Trung trong bệnh viện không?',
            'Ở bệnh viện có nhân viên tên Thành hay không?',
            'Có bác sĩ tên Thanh không?',
        ):
            with self.subTest(question=question):
                self.assertTrue(staff_question_candidate(question))

    def test_unrelated_hospital_questions_stay_with_ai(self):
        for question in ('Xe lăn ở đâu?', 'Phòng điều dưỡng làm gì?'):
            with self.subTest(question=question):
                self.assertFalse(staff_question_candidate(question))

    def test_named_staff_navigation_uses_live_duty(self):
        self.assertTrue(staff_question_candidate('Dẫn tôi đến phòng của nhân viên Trung'))
        self.assertTrue(staff_question_candidate('Đưa tôi đến phòng của bác sĩ Thanh'))
        self.assertFalse(staff_question_candidate('Dẫn tôi đến phòng lễ tân'))


if __name__ == '__main__':
    unittest.main()
