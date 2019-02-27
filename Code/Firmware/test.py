import unittest
import Body
import logging

class BodyTestCase(unittest.TestCase):
    def setUp(self):
        self.body = Body.SwarmBody()

    def test_initialise(self):
        self.assertEqual(self.body._get_pos, 1)
        self.assertIsNot(self.body.gyro_data, 0)
        logging.log(22, "[+] Body Initialised")


    def test_position(self):
        if self.body._get_pos and self.body.gyro_data != 0:
            pos = self.body.get_pos(1)
        self.assertIsNotNone(pos[0])
        self.assertIsNotNone(pos[1])
        logging.log(22, "[+] Position acquired")

