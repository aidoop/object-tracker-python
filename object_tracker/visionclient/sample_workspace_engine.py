import sys
import json
from random import choice, random
import time
from operato_vision import Client


def main(argv):
    workspaceName = argv[1]

    client = Client('http://localhost:3000', 'system')
    client.signin('admin@hatiolab.com', 'admin')

    while(True):
        update(client)
        time.sleep(1)


def update(client):
    roiA = choice(["A", "B", None, None])
    roiB = choice(["A", "B", None, None])

    if roiA == None:
        poseA = None
    else:
        poseA = {
            "x": 1.0 + random() * 1.2,
            "y": 2.2 + random() * 1.2,
            "z": 3.4 + random() * 1.2,
            "u": 4.4 + random() * 1.2,
            "v": 7.9 + random() * 1.2,
            "w": 9.6 + random() * 1.2
        }

    if roiB == None:
        poseB = None
    else:
        poseB = {
            "x": 1.0 + random() * 1.2,
            "y": 2.2 + random() * 1.2,
            "z": 3.4 + random() * 1.2,
            "u": 4.4 + random() * 1.2,
            "v": 7.9 + random() * 1.2,
            "w": 9.6 + random() * 1.2
        }

    status = {
        "objectStatus": [{
            "id": "obj",
            "state": {
                "roi": roiA,
                "pose": poseA
            }
        }, {
            "id": "obj2",
            "state": {
                "roi": roiB,
                "pose": poseB
            }
        }]
    }

    result = client.update_tracking_workspace_status(
        name='workspace', status=status)

    print(result)


if __name__ == "__main__":
    main(sys.argv)
