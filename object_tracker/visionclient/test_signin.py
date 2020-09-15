import unittest

from operato_vision import Client


class TestGetTrackingCamera(unittest.TestCase):
    def test_signin(self):
        """
        Operato 서버에 인증 과정 API
        """
        client = Client('http://localhost:3000', 'system')
        client.signin('admin@hatiolab.com', 'admin')

        self.assertIsNotNone(client.access_token)


if __name__ == '__main__':
    unittest.main()
