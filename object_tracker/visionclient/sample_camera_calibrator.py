import sys
import json


def main(argv):
    cameraName = argv[1]

    cameraParameter = {
        "distortionCoefficient": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        "cameraMatrix": {
            "rows": 3,
            "columns": 3,
            "data": [0.0, 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7,
                     8.8]
        }
    }

    print(json.dumps(cameraParameter))


if __name__ == "__main__":
    main(sys.argv)
