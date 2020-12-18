import sys
import json
from random import choice, random
import time
from operato_vision import Client


def main(argv):
    workspaceName = argv[1]

    client = Client('http://localhost:3000', 'system')
    client.signin('admin@hatiolab.com', 'admin')

    client.robot_go_home(name='robot01')


if __name__ == "__main__":
    main(sys.argv)
