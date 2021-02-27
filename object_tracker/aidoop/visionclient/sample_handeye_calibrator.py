import sys
import json


def main(argv):
    cameraName = argv[1]

    handeyeMatrix = {
        "rows": 4,
        "columns": 4,
        "data": [0.0, 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7,
                 8.8, 9.9, 10.1, 11.2, 12.3, 12.4, 12.5, 12.60]
    }

    print(json.dumps(handeyeMatrix))


if __name__ == "__main__":
    main(sys.argv)
