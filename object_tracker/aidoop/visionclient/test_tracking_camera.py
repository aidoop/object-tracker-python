import unittest

from operato_vision import Client


class TestGetTrackingCamera(unittest.TestCase):
    def test_get_tracking_camera(self):
        """
        트래킹 카메라 리스트를 조회하는 API 테스트
        """
        client = Client('http://localhost:3000', 'system')
        client.signin('admin@hatiolab.com', 'admin')

        cameras = client.get_tracking_cameras()['items']

        for camera in cameras:
            name = camera['name']
            print(name, '\n', client.get_tracking_camera(name=name), '\n')

        self.assertEqual(len(cameras), 3)


if __name__ == '__main__':
    unittest.main()
