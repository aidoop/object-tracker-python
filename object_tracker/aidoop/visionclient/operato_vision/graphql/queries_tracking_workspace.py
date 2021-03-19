from .queries import QUERIES, MUTATION

QUERY_TRACKING_WORKSPACE = "trackingWorkspace"
QUERY_TRACKING_WORKSPACES = "trackingWorkspaces"
MUTATION_UPDATE_WORKSPACE_STATUS = "updateTrackingWorkspaceStatus"

QUERIES[
    QUERY_TRACKING_WORKSPACE
] = """
query trackingWorkspace($name: String!) {
    trackingWorkspace(name:$name) {
        id
        name
        domain {
            name
        }
        description
        type
        endpoint
        active
        checkVideoStream
        detectionMethod
        robotArms {
            id
            name
        }
        trackingCameras {
            id
            name
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
        }
        trackableObjects {
            id
            name
        }
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
    QUERY_TRACKING_WORKSPACES
] = """
query {
    trackingWorkspaces {
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
            checkVideoStream
            detectionMethod
            robotArms {
                id
                name
            }
            trackingCameras {
                id
                name
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
            }
            trackableObjects {
                id
                name
            }
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

MUTATION[
    MUTATION_UPDATE_WORKSPACE_STATUS
] = """
mutation updateTrackingWorkspaceStatus($name: String!, $status: TrackingWorkspaceStatusInput!) {
    updateTrackingWorkspaceStatus(name:$name, status:$status)
}
"""
