#!/usr/bin/env python3
import os
import rospy

from std_msgs.msg import String
from k4r_data_msgs.srv import \
    ShelfCreate, ShelfCreateResponse, \
    ShelfDelete, ShelfDeleteResponse, \
    ShelfGet, ShelfGetResponse, \
    ShelfList, ShelfListResponse, \
    ShelfUpdate, ShelfUpdateResponse, \
    ShelfOverwrite, ShelfOverwriteResponse
from k4r_data_msgs.msg import Shelf

# This file has initially been generated using ROSCRUD

# ROS CRUD services
SHELF_CREATE = '/shelf/create'
SHELF_DELETE = '/shelf/delete'
SHELF_GET = '/shelf/get'
SHELF_LIST = '/shelf/list'
SHELF_UPDATE = '/shelf/update'
SHELF_OVERWRITE = '/shelf/overwrite'


# ROS Topics to inform data change
SHELF_ADDED = '/shelf/added'
SHELF_CHANGED = '/shelf/changed'
SHELF_REMOVED = '/shelf/removed'


class ShelfNode:
    def __init__(self):
        self.name = self.__class__.__name__
        
        # all shelf by shelf_id
        # type: {shelf_id: Shelf}
        self.data = {}

        # overwrite shelf in data if another shelf
        # with shelf_id is received or log an error and ignore
        # the new shelf_id
        self.allow_overwrite = rospy.get_param('~allow_overwrite', False)
        
        # name of the id
        # type: string
        self.id_name = 'shelf_id'
        
        # setup services for data manipulation
        self.init_ros()

    def init_ros(self):
        added_param = rospy.get_param(
            '~shelf_added', SHELF_ADDED)
        self.added_pub = rospy.Publisher(
            added_param, Shelf, queue_size=10)
        changed_param = rospy.get_param(
            '~shelf_changed', SHELF_CHANGED)
        self.changed_pub = rospy.Publisher(
            changed_param, Shelf, queue_size=10)
        removed_param = rospy.get_param(
            '~shelf_removed', SHELF_REMOVED)
        self.removed_pub = rospy.Publisher(
            removed_param, String, queue_size=10)
        
        create_param = rospy.get_param(
            '~shelf_create', SHELF_CREATE)
        self.create_srv = rospy.Service(
            create_param, ShelfCreate, self.on_create)
        delete_param = rospy.get_param(
            '~shelf_delete', SHELF_DELETE)
        self.delete_srv = rospy.Service(
            delete_param, ShelfDelete, self.on_delete)
        get_param = rospy.get_param(
            '~shelf_get', SHELF_GET)
        self.get_srv = rospy.Service(
            get_param, ShelfGet, self.on_get)
        list_param = rospy.get_param(
            '~shelf_list', SHELF_LIST)
        self.list_srv = rospy.Service(
            list_param, ShelfList, self.on_list)
        update_param = rospy.get_param(
            '~shelf_update', SHELF_UPDATE)
        self.update_srv = rospy.Service(
            update_param, ShelfUpdate, self.on_update)
        overwrite_param = rospy.get_param(
            '~shelf_overwrite', SHELF_OVERWRITE)
        self.overwrite_srv = rospy.Service(
            overwrite_param, ShelfOverwrite, self.on_overwrite)
        


    def on_create(self, msg):
        elem = msg.shelf
        _shelf_id = elem.shelf_id
        if _shelf_id in self.data and not self.allow_overwrite:
            rospy.logerr(
                '[ShelfNode] Can not create, shelf_id ' +
                _shelf_id + ' already exists.')
            return ShelfCreateResponse(success=False)
        self.data[_shelf_id] = elem
        self.added_pub.publish(elem)
        return ShelfCreateResponse(success=True)

    def on_delete(self, msg):
        _shelf_id = msg.shelf_id
        if _shelf_id not in self.data:
            rospy.logerr(
                '[ShelfNode] Can not delete, shelf_id ' +
                _shelf_id + ' does not exist.')
            return ShelfDeleteResponse(success=False)
        del self.data[_shelf_id]
        self.removed_pub.publish(_shelf_id)
        return ShelfDeleteResponse(success=True)

    def on_get(self, msg):
        if msg.shelf_id in self.data:
            return ShelfGetResponse(shelf=self.data[msg.shelf_id])
        else:
            rospy.loginfo(
                '[ShelfNode] ' + msg.shelf_id + ' does not exist!')
            return ShelfGetResponse()

    def on_list(self, msg):
        return ShelfListResponse(shelf=self.data.values())

    def on_update(self, msg):
        elem = msg.shelf
        _shelf_id = elem.shelf_id
        if _shelf_id not in self.data:
            rospy.logerr(
                '[ShelfNode] Can not update, shelf_id ' +
                _shelf_id + ' does not exist.')
            return ShelfUpdateResponse(success=False)
        self.data[_shelf_id] = elem
        self.changed_pub.publish(elem)
        return ShelfUpdateResponse(success=True)

    def on_overwrite(self, msg):
        """allow overwrite shelf in self.data"""
        self.allow_overwrite = msg.allow_overwrite
        return ShelfOverwriteResponse()


    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('shelf_node', anonymous=True)
    node = ShelfNode()
    rospy.loginfo('Shelf node started')
    node.spin()