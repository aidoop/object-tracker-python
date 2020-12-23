import sys
import json
from random import choice, random
import time
from operato_vision import Client


def main(argv):
    workspaceName = argv[1]

    client = Client('http://localhost:3000', 'system')
    client.signin('admin@hatiolab.com', 'admin')

    # client.robot_go_home(name='robot01')

    # client.robot_task_moveby(
    #     name='robot01', pose={'x': 0.0, 'y': 0.01, 'z': 0.0, 'u': 0.0, 'v': 0.0, 'w': 0.0})

    print(client.get_robot_status(name='robot01'))
    print(client.get_robot_status(name='robot01')['moveFinished'])


if __name__ == "__main__":
    main(sys.argv)
