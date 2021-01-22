import asyncio
import json
import requests
import asyncio

from gql import gql, Client as GqlClient
from gql.transport.requests import RequestsHTTPTransport

from .graphql import graphql_query, graphql_mutation
from .graphql import (
    QUERY_TRACKING_WORKSPACE,
    QUERY_TRACKING_WORKSPACES,
    QUERY_TRACKING_CAMERA,
    QUERY_TRACKING_CAMERAS,
)
from .graphql import MUTATION_UPDATE_WORKSPACE_STATUS
from .graphql import (
    QUERY_TRACKABLE_OBJECT,
    QUERY_TRACKABLE_OBJECTS,
    QUERY_ROBOT_ARM,
    QUERY_ROBOT_ARMS,
    QUERY_ROBOT_ARM_POSE,
    MUTATION_ROBOT_ARM_POSE,
)
from .graphql import (
    MUTATION_ROBOTAPI_GOHOME,
    MUTATION_ROBOTAPI_TASKMOVEBY,
    QUERY_ROBOTAPI_GET_STATUS,
    MUTATION_ROBOTAPI_TASKMOVEBYNOWAIT,
)


class Client:
    def __init__(self, endpoint, domain):
        self.endpoint = endpoint
        self.domain = domain
        self.client = None

    def signin(self, email, password):

        url = "{0}/auth/signin".format(self.endpoint)
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json"
            # "x-only-token": "true"
        }
        security = {"email": email, "password": password}

        response = requests.post(url, headers=headers, json=security)

        if response.status_code == 200:
            self.access_token = response.text
        else:
            self.access_token = None

        reqHeaders = {
            "authorization": self.access_token,
            # 'x-things-factory-domain': self.domain
        }

        _transport = RequestsHTTPTransport(
            url="{0}/graphql".format(self.endpoint),
            headers=reqHeaders,
            use_json=True,
        )

        self.client = GqlClient(
            transport=_transport,
            fetch_schema_from_transport=True,
        )

        return

    @graphql_query(QUERY_TRACKING_WORKSPACES)
    def get_tracking_workspaces(self):
        pass

    @graphql_query(QUERY_TRACKING_WORKSPACE, ["name"])
    def get_tracking_workspace(self, name):
        pass

    @graphql_query(QUERY_TRACKING_CAMERAS)
    def get_tracking_cameras(self):
        pass

    @graphql_query(QUERY_TRACKING_CAMERA, ["name"])
    def get_tracking_camera(self, name):
        pass

    @graphql_query(QUERY_TRACKABLE_OBJECTS)
    def get_trackable_objects(self):
        pass

    @graphql_query(QUERY_TRACKABLE_OBJECT, ["name"])
    def get_trackable_object(self, name):
        pass

    @graphql_query(QUERY_ROBOT_ARMS)
    def get_robot_arms(self):
        pass

    @graphql_query(QUERY_ROBOT_ARM, ["name"])
    def get_robot_arm(self, name):
        pass

    @graphql_query(QUERY_ROBOT_ARM_POSE, ["name"])
    def get_robot_arm_pose(self, name):
        pass

    @graphql_mutation(MUTATION_ROBOT_ARM_POSE, ["name", "pose"])
    def set_robot_arm_pose(self, name, pose):
        pass

    @graphql_mutation(MUTATION_UPDATE_WORKSPACE_STATUS, ["name", "status"])
    def update_tracking_workspace_status(self, name, status):
        pass

    #####################################
    # Robot API

    @graphql_mutation(MUTATION_ROBOTAPI_GOHOME, ["name"])
    def robot_go_home(self, name):
        pass

    @graphql_mutation(MUTATION_ROBOTAPI_TASKMOVEBY, ["name", "pose"])
    def robot_task_moveby(self, name, pose):
        pass

    @graphql_mutation(MUTATION_ROBOTAPI_TASKMOVEBYNOWAIT, ["name", "pose"])
    def robot_task_moveby_nowait(self, name, pose):
        pass

    @graphql_query(QUERY_ROBOTAPI_GET_STATUS, ["name"])
    def get_robot_status(self, name):
        pass
