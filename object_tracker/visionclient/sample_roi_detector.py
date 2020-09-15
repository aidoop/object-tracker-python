import sys
import json


def main(argv):
    cameraName = argv[1]

    ROIs = [{
        "id": cameraName + 'A',
        "region": {
            "lt": {
                "x": 103,
                "y": 100
            },
            "rb": {
                "x": 203,
                "y": 200
            }
        }
    }, {
        "id": cameraName + 'B',
        "region": {
            "lt": {
                "x": 105,
                "y": 100
            },
            "rb": {
                "x": 205,
                "y": 200
            }
        }
    }]

    print(json.dumps(ROIs))


if __name__ == "__main__":
    main(sys.argv)
