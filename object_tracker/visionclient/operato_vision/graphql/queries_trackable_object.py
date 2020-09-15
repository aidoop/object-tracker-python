from .queries import QUERIES

QUERY_TRACKABLE_OBJECT = 'trackableObject'
QUERY_TRACKABLE_OBJECTS = 'trackableObjects'

QUERIES[QUERY_TRACKABLE_OBJECT] = '''
query trackableObject($name: String!) {
    trackableObject(name:$name) {
        id
        name
        domain {
            name
        }
        description
        type
        endpoint
        active
        poiOffset {
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

QUERIES[QUERY_TRACKABLE_OBJECTS] = '''
query {
    trackableObjects {
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
            poiOffset {
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
