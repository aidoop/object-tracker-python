from .queries import QUERIES, MUTATION

QUERY_ROBOT_ARM = 'robotArm'
QUERY_ROBOT_ARMS = 'robotArms'
QUERY_ROBOT_ARM_POSE = 'robotArmPose'
MUTATION_ROBOT_ARM_POSE = 'updateRobotArmPose'

QUERIES[QUERY_ROBOT_ARM] = '''
query robotArm($name: String!) {
    robotArm(name:$name) {
        id
        name
        domain {
            name
        }
        description
        type
        endpoint
        active
        markerOffset {
            x
            y
            z
            u
            v
            w
        }
        toolOffset {
            x
            y
            z
            u
            v
            w
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
'''

QUERIES[QUERY_ROBOT_ARMS] = '''
query {
    robotArms {
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
            toolOffset {
                x
                y
                z
                u
                v
                w
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
'''


QUERIES[QUERY_ROBOT_ARM_POSE] = '''
query robotArmPose($name: String!) {
    robotArmPose(name:$name) {
        x
        y
        z
        u
        w
        v
    }
}
'''


MUTATION[MUTATION_ROBOT_ARM_POSE] = '''
mutation updateRobotArmPose($name: String!, $pose: PoseInput!) {
    updateRobotArmPose(name:$name, pose:$pose) {
        id
        name
    }
}
'''
