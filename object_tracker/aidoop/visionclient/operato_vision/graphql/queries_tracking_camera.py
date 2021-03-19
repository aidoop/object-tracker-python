from .queries import QUERIES

QUERY_TRACKING_CAMERA = "trackingCamera"
QUERY_TRACKING_CAMERAS = "trackingCameras"

QUERIES[
    QUERY_TRACKING_CAMERA
] = """
query trackingCamera($name: String!) {
    trackingCamera(name:$name) {
        id
        name
        domain {
            name
        }
        description
        type
        endpoint
        active
        config
        baseRobotArm {
            id
            name
        }
        cameraMatrix {
            rows
            columns
            data
        }
        distortionCoefficient
        handEyeConfig {
            mode
            autoTotalIter
            autoMoveXyz
            autoMoveUvw
        }
        handEyeMatrix {
            rows
            columns
            data
        }
        rois {
            id
            region {
                lt {
                    x
                    y
                }
                rb {
                    x
                    y
                }
            }
        }
        camObjOffset {
            x
            y
            z
            u
            v
            w
        }    
        width
        height
        updater {
            email
        }
        creator {
            email
        }
        updatedAt
        createdAt
    }
}
"""

QUERIES[
    QUERY_TRACKING_CAMERAS
] = """
query {
    trackingCameras {
        items {
            id
            name
            domain {
                name
            }
            description
            type
            endpoint
            active
            config
            baseRobotArm {
                id
                name
            }
            cameraMatrix {
                rows
                columns
                data
            }
            distortionCoefficient
            handEyeConfig {
                mode
                autoTotalIter
                autoMoveXyz
                autoMoveUvw
            }
            handEyeMatrix {
                rows
                columns
                data
            }
            rois {
                id
                region {
                    lt {
                        x
                        y
                    }
                    rb {
                        x
                        y
                    }
                }
            }
            camObjOffset {
                x
                y
                z
                u
                v
                w
            }    
            width
            height
            updater {
                email
            }
            creator {
                email
            }
            updatedAt
            createdAt
        }
        total
    }
}
"""
