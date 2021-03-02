import unittest

from operato_vision import Client


class TestGetTrackableObject(unittest.TestCase):
    def test_get_trackable_object(self):
        """
        트래킹 카메라 리스트를 조회하는 API 테스트
        """
        client = Client('http://localhost:3000', 'system')
        client.signin('admin@hatiolab.com', 'admin')

        obzects = client.get_trackable_objects()['items']

        for obzect in obzects:
            name = obzect['name']
            print(name, '\n', client.get_trackable_object(name=name), '\n')

        self.assertEqual(len(obzects), 0)


if __name__ == '__main__':
    unittest.main()
