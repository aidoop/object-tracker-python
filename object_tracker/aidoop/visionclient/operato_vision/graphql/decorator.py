from gql import gql, Client
from .queries import QUERIES, MUTATION


def graphql_query(query, varnames=[]):
    def decorate(func):
        def decorated(self, **kwargs):

            variables = func(self, **kwargs)

            q = gql(QUERIES[query])

            variables = {}
            i = 0

            for varname in varnames:
                variables[varname] = kwargs[varname]
                i = i + 1

            return self.client.execute(q, variable_values=variables)[query]
        return decorated
    return decorate


def graphql_mutation(mutation, varnames=[]):
    def decorate(func):
        def decorated(self, **kwargs):

            variables = func(self, **kwargs)

            m = gql(MUTATION[mutation])

            variables = {}
            i = 0

            for varname in varnames:
                variables[varname] = kwargs[varname]
                i = i + 1

            return self.client.execute(m, variable_values=variables)[mutation]
        return decorated
    return decorate
