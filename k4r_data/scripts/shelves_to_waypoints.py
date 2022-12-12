#!/usr/bin/env python3
# create waypoints from shelves
import os
import math
import rospy

# ROS basic datatypes
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import String

# tf helper functions
from tf.transformations import \
    euler_from_quaternion

# custom ROS-datatypes/Messages
from k4r_data_msgs.msg import Shelf
from agent_map_msgs.msg import Map, Waypoint
# custom ROS-Services to get shelves and add waypoints
from k4r_data_msgs.srv import \
    ShelfList, ShelfListRequest, \
    ShelfGet, ShelfGetRequest
from agent_map_msgs.srv import \
    WaypointAdd, WaypointAddRequest, \
    WaypointAddList, WaypointAddListRequest, \
    WaypointDelete, WaypointDeleteRequest

# ROSCrud Topics and services for waypoints, 
# See agent_map/map_waypoint_wrapper.py
WAYPOINT_ADD = '/map/waypoint/add'
WAYPOINT_ADD_LIST = '/map/waypoint/add_list'
WAYPOINT_DELETE = '/map/waypoint/delete'

# ROSCrud Topics and Services for shelves, see shelf.py
SHELF_GET = '/shelf/get'
SHELF_LIST = '/shelf/list'
SHELF_ADDED = '/shelf/added'
SHELF_CHANGED = '/shelf/changed'
SHELF_REMOVED = '/shelf/removed'

# minimum distance between robot and shelf
DINSTANCE_TO_SHELF = 0.3
# width of robot including trolley
ROBOT_SIZE = 0.7

# assume a shelf has a depth of 0.5 meter
DEFAULT_DEPTH = 0.5


class ShelvesToWaypointNode:
    def __init__(self) -> None:
        self.name = self.__class__.__name__
        self.init_ros()

    def init_ros(self) -> None:
        # init ROS topics and services
        # get shelves...
        shelf_list_param = rospy.get_param('~shelf_list', SHELF_LIST)
        rospy.wait_for_service(shelf_list_param)
        self.shelf_list_srv = rospy.ServiceProxy(shelf_list_param, ShelfList)

        added_param = rospy.get_param('~shelf_added', SHELF_ADDED)
        rospy.Subscriber(added_param, Shelf, self.on_shelf_added)
        changed_param = rospy.get_param('~shelf_changed', SHELF_CHANGED)
        rospy.Subscriber(changed_param, Shelf, self.on_shelf_changed)
        removed_param = rospy.get_param('~shelf_removed', SHELF_REMOVED)
        rospy.Subscriber(removed_param, String, self.on_shelf_removed)

        # ...and turn them into waypoints
        add_wp_service = rospy.get_param('~waypoint_add', WAYPOINT_ADD)
        rospy.wait_for_service(add_wp_service)
        self.add_wp = rospy.ServiceProxy(add_wp_service, WaypointAdd)

        list_wp = rospy.get_param('~waypoint_add_list', WAYPOINT_ADD_LIST)
        rospy.wait_for_service(list_wp)
        self.add_wp_list = rospy.ServiceProxy(list_wp, WaypointAddList)
        
        del_wp_service = rospy.get_param('~waypoint_delete', WAYPOINT_DELETE)
        rospy.wait_for_service(del_wp_service)
        self.del_wp = rospy.ServiceProxy(del_wp_service, WaypointDelete)

        self.list_shelves()

    def list_shelves(self) -> None:
        msg = self.shelf_list_srv(ShelfListRequest())
        waypoints = []
        if len(msg.shelf) == 0:
            rospy.logerr(f'[{self.name}] No shelves stored yet!')
            return
        shelves_by_map = {}
        for shelf in msg.shelf:
            wp = self.create_waypoint_from_shelf(shelf)
            map_name = shelf.map_name
            shelves_by_map.setdefault(map_name, [])
            shelves_by_map[map_name].append(wp)
        for map_name, wps in shelves_by_map.items():
            # append waypoints to existing map or create a new one
            resp = self.add_wp_list(
                WaypointAddListRequest(
                    map_name=map_name,
                    waypoints=wps
                )
            )
            if not resp.success:
                rospy.logerr(
                    f'[{self.name}] can not add waypoints {wps} '
                    f'to map {map_name}')

    def on_shelf_added(self, shelf: Shelf) -> None:
        wp = self.create_waypoint_from_shelf(shelf)
        # append waypoints to existing map or create a new one
        self.add_wp(
            WaypointAddRequest(
                map_name=shelf.map_name,
                waypoint=wp
            )
        )

    def create_waypoint_from_shelf(self, shelf: Shelf) -> Waypoint:
        waypoint = Waypoint(
            name=shelf.shelf_id,
            data=[KeyValue(key='confirmation_needed', value='True')])
        depth = shelf.depth if shelf.depth > 0 else DEFAULT_DEPTH
        # calculate position in front of the shelf.
        distance_to_shelf_border = depth / 2
        pose = shelf.pose
        orientation = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]

        # euler = roll, pitch, yaw in radians
        euler = euler_from_quaternion(orientation)
        
        angle = euler[2] + (math.pi / 2)

        # calculate position infront of shelf as new waypoint.
        robot_half = ROBOT_SIZE / 2
        dist_x = math.cos(angle) * (DINSTANCE_TO_SHELF + depth + robot_half)
        dist_y = math.sin(angle) * (DINSTANCE_TO_SHELF + depth + robot_half)
        pose.position.x -= dist_x
        pose.position.y -= dist_y

        waypoint.pose = pose
        return waypoint


    def on_shelf_changed(self, shelf: Shelf) -> None:
        # TODO!
        pass

    def on_shelf_removed(self, shelf_id: String) -> None:
        shelf = self.shelf_get(ShelfGetRequest(shelf_id=shelf_id))
        wp = create_waypoint_from_shelf(shelf)
        resp = self.del_wp(WaypointDeleteRequest(
            map_name=shelf.map_name,
            waypoint_name=wp.name))
        if not resp.success:
            rospy.logerr(
                f'[{self.name}] Error deleting waypoint "{wp.name}" '
                f'in map "{shelf.map_name}"')


    def spin(self) -> None:
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('shelves_to_waypoints_node', anonymous=True)
    node = ShelvesToWaypointNode()
    node.spin()