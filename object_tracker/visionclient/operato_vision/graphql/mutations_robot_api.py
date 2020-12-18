from .queries import QUERIES, MUTATION

MUTATION_ROBOTAPI_GOHOME = 'robotArmGoHome'

MUTATION[MUTATION_ROBOTAPI_GOHOME] = '''
mutation robotArmGoHome($name: String!) {
    robotArmGoHome(name:$name)
}
'''
