from .queries import QUERIES, MUTATION

QUERY_ROBOTAPI_GET_STATUS = 'robotArmGetStatus'

MUTATION_ROBOTAPI_GOHOME = 'robotArmGoHome'
MUTATION_ROBOTAPI_TASKMOVEBY = 'robotArmTaskMoveBy'
MUTATION_ROBOTAPI_TASKMOVEBYNOWAIT = 'robotArmTaskMoveByNoWait'


QUERIES[QUERY_ROBOTAPI_GET_STATUS] = '''
query robotArmGetStatus($name: String!) {
    robotArmGetStatus(name:$name) {
        running
        ready
        busy
        moveFinished
        home
        zero
        teachingMode
        error
        collided
        emergency         
    }
}
'''


MUTATION[MUTATION_ROBOTAPI_GOHOME] = '''
mutation robotArmGoHome($name: String!) {
    robotArmGoHome(name:$name)
}
'''

MUTATION[MUTATION_ROBOTAPI_TASKMOVEBY] = '''
mutation robotArmTaskMoveBy($name: String!, $pose: PoseInput!) {
    robotArmTaskMoveBy(name:$name, pose:$pose) {
        x
        y
        z
        u
        v
        w
    }
}
'''

MUTATION[MUTATION_ROBOTAPI_TASKMOVEBYNOWAIT] = '''
mutation robotArmTaskMoveByNoWait($name: String!, $pose: PoseInput!) {
    robotArmTaskMoveByNoWait(name:$name, pose:$pose) {
        x
        y
        z
        u
        v
        w
    }
}
'''
