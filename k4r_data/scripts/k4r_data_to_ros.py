#!/usr/bin/env python3
# entrypoint for all external data into the system!
# load data from data provider and publish on ROS topic
# run on the K4R-Platform

import rospy
import json
# local imports
from utils import orientation_from_list
from data_provider import get_data_provider_class
from data_provider.data import DataProvider

# ROS messages and services  - basics
from geometry_msgs.msg import Pose

# ROS messages and services - agent manager / agent map
from agent_map_msgs.srv import \
    AgentMapList, AgentMapListRequest, \
    MapCreate, MapCreateRequest
from agent_manager_msgs.msg import Agent
from agent_map_msgs.msg import AgentMap, Map


# ROS messages and services - k4r-data
from k4r_data_msgs.msg import Shelf, Product
from k4r_data_msgs.srv import \
    ProductCreate, ProductCreateRequest, \
    ShelfCreate, ShelfCreateRequest

# Create stores (map), shelves and products
MAP_CREATE = '/map/create'
SHELF_CREATE = '/shelf/create'
PRODUCT_CREATE = '/product/create'

# get list of agents from agent manager
AGENT_MAP_LIST = '/agent_map/list'
AGENT_MAP_ADDED = '/agent_map/added'
AGENT_MAP_CHANGED = '/agent_map/changed'

DEFAULT_MAP = '01_Uni-Bremen'


def get_pose(position, orientation, quaternion, radians):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    orientation = orientation_from_list(orientation, quaternion, radians)
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose


def product_to_ros(data_product, map_name):
    ros_product = Product()
    ros_product.map_name = map_name
    ros_product.gtin = data_product.gtin
    ros_product.product_id = data_product.product_id
    ros_product.name = data_product.name
    ros_product.shelf_ids = data_product.shelf_ids
    ros_product.pose = get_pose(
        data_product.position, data_product.orientation, 
        data_product.quaternion, data_product.radians)
    return ros_product


def shelf_to_ros(data_shelf, map_name):
    # convert data_shelf (data_provider.Shelf) to k4r_data_msgs/Shelf
    ros_shelf = Shelf()
    ros_shelf.map_name = map_name
    ros_shelf.shelf_id = data_shelf.shelf_id
    ros_shelf.model = data_shelf.model
    ros_shelf.pose = get_pose(
        data_shelf.position, data_shelf.orientation, 
        data_shelf.quaternion, data_shelf.radians)
    ros_shelf.depth = data_shelf.depth
    ros_shelf.width = data_shelf.width
    ros_shelf.height = data_shelf.height
    return ros_shelf


class DataLoader:
    def __init__(self):
        self.name = self.__class__.__name__
        self.known_maps = []
        self.get_data_provider()
        self.init_services()
        
        # get all stores/maps
        self.get_store_list()
        # get all agents assigned to maps
        #self.get_agent_list()

    def get_data_provider(self):
        """Get Data Provider instances."""
        dp_types = rospy.get_param('~data_provider', '["static"]')
        if dp_types.startswith('"['):
            dp_types = json.loads(dp_types[1:-1].replace('\'', '"'))
        elif dp_types.startswith('['):
            dp_types = json.loads(dp_types.replace('\'', '"'))
        else:
            dp_types = [dp_types]
        
        self.data_provider = []
        for data_provider_type in dp_types:
            rospy.loginfo(
                f'[{self.name}] Using Data Provider {data_provider_type}')
            data_clz = get_data_provider_class(data_provider_type)
            self.data_provider.append(data_clz())

    def _init_service_proxy(self, name, default_topic, clz):
        topic_name = rospy.get_param(f'~{name}', default_topic)
        rospy.wait_for_service(topic_name, 300)
        return rospy.ServiceProxy(topic_name, clz)

    def _subscribe(self, name, default_topic, clz, callback):
        topic_name = rospy.get_param(f'~{name}', default_topic)
        return rospy.Subscriber(topic_name, clz, callback)

    def init_services(self):
        self.add_product = self._init_service_proxy(
            'add_product', PRODUCT_CREATE, ProductCreate)
        self.add_shelf = self._init_service_proxy(
            'add_shelf', SHELF_CREATE, ShelfCreate)
        self.agent_list = self._init_service_proxy(
            'agent_map_list', AGENT_MAP_LIST, AgentMapList)
        self.add_map = self._init_service_proxy(
            'add_map', MAP_CREATE, MapCreate)

        self._subscribe(
            'agent_map_added', AGENT_MAP_ADDED, 
            AgentMap, self.on_agent_map_added)
        self._subscribe(
            'agent_map_changed', AGENT_MAP_CHANGED, 
            AgentMap, self.on_agent_map_changed)

    def get_agent_list(self):
        # get list of agents and load data from the maps they are
        # assigned to.
        req = self.agent_list(AgentMapListRequest())
        for agent in req.agent:
            self.load(agent)

    def get_store_list(self):
        num_stores = 0
        for data_provider in self.data_provider:
            for store in data_provider.get_stores():
                # create Map-msg from store data
                _map = Map(
                    name=f'{store.id}_{store.storeName}',
                    waypoints=[]  # start with a list of empty waypoints
                )
                # call create-map service
                self.add_map(_map)
        if num_stores > 0:
            rospy.loginfo(f'[{self.name}] imported {num_stores} stores!')

    def on_agent_map_added(self, agent_map: AgentMap):
        rospy.loginfo(f'AGENT MAP ADDED! {agent_map.map_name}')
        self.load(agent_map)


    def on_agent_map_changed(self, agent_map: AgentMap):
        rospy.loginfo(f'AGENT MAP CHANGED! {agent_map.map_name}')
        self.load(agent_map)


    def load(self, agent_map: AgentMap):
        map_name = agent_map.map_name
        if map_name in self.known_maps:
            return self.known_maps[map_name]

        for data_provider in self.data_provider:
            # the first data provider that knows the requested map
            # will be used to request data from its data source and publish
            # it to the ros-network.
            if data_provider.knows_store(map_name):
                rospy.loginfo(
                    f'Using data Provider {data_provider.name} '
                    f'for {map_name}')
                return self.from_data_provider(data_provider, map_name)

    def from_data_provider(self, data_provider: DataProvider, map_name: str):
        # the shelves will be turned into waypoints in shelves_to_waypoints.py
        # The map_waypoint_wrapper will create the map if it does not exist!
        data_shelves = data_provider.get_shelves(map_name)
        for data_shelf in data_shelves:
            ros_shelf = shelf_to_ros(data_shelf, map_name)
            self.add_shelf(
                ShelfCreateRequest(shelf=ros_shelf)
            )

        # load products
        data_products = data_provider.get_products(map_name)
        print('*'*20)
        print(data_products)
        for data_prod in data_products:
            ros_prod = product_to_ros(data_prod, map_name)
            self.add_product(
                ProductCreateRequest(product=ros_prod)
            )

        rospy.loginfo(
            f'[{self.name}] loaded {len(data_shelves)} shelves and '
            f'{len(data_products)} products to map "{map_name}" from '
            f'data provider "{data_provider.name}"')



    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("data_loader", log_level=rospy.INFO)
    loader = DataLoader()
    loader.spin()
