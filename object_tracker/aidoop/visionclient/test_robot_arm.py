import unittest

from operato_vision import Client


class TestGetRobotArms(unittest.TestCase):
    def test_get_robot_arms(self):
        """
        트래킹 카메라 리스트를 조회하는 API 테스트
        """
        client = Client('http://localhost:3000', 'system')
        client.signin('admin@hatiolab.com', 'admin')

        robot_arms = client.get_robot_arms()['items']

        for robot_arm in robot_arms:
            name = robot_arm['name']
            print(name, '\n', client.get_robot_arm(name=name), '\n')
            print(name, '\n', client.get_robot_arm_pose(name=name), '\n')

        self.assertEqual(len(robot_arms), 0)


if __name__ == '__main__':
    unittest.main()
