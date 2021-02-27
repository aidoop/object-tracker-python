import unittest

from operato_vision import Client


class TestGetTrackingWorkspaces(unittest.TestCase):
    def test_get_tracking_workspaces(self):
        """
        트래킹 카메라 리스트를 조회하는 API 테스트
        """
        client = Client('http://localhost:3000', 'system')
        client.signin('admin@hatiolab.com', 'admin')

        workspaces = client.get_tracking_workspaces()['items']
        print('\n\n', workspaces, '\n')

        for workspace in workspaces:
            name = workspace['name']
            print(name, '\n\n', client.get_tracking_workspace(name=name), '\n')

        self.assertEqual(len(workspaces), 1)


if __name__ == '__main__':
    unittest.main()
