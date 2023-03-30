#!/usr/bin/env python3
import os
import rospy

from std_msgs.msg import String
from k4r_data_msgs.srv import \
    ProductCreate, ProductCreateResponse, \
    ProductDelete, ProductDeleteResponse, \
    ProductGet, ProductGetResponse, \
    ProductList, ProductListResponse, \
    ProductUpdate, ProductUpdateResponse, \
    ProductOverwrite, ProductOverwriteResponse
from k4r_data_msgs.msg import Product

# This file has initially been generated using ROSCRUD

# ROS CRUD services
PRODUCT_CREATE = '/product/create'
PRODUCT_DELETE = '/product/delete'
PRODUCT_GET = '/product/get'
PRODUCT_LIST = '/product/list'
PRODUCT_UPDATE = '/product/update'
PRODUCT_OVERWRITE = '/product/overwrite'


# ROS Topics to inform data change
PRODUCT_ADDED = '/product/added'
PRODUCT_CHANGED = '/product/changed'
PRODUCT_REMOVED = '/product/removed'


class ProductNode:
    def __init__(self):
        self.name = self.__class__.__name__
        
        # all product by product_id
        # type: {product_id: Product}
        self.data = {}

        # overwrite product in data if another product
        # with product_id is received or log an error and ignore
        # the new product_id
        self.allow_overwrite = rospy.get_param('~allow_overwrite', False)
        
        # name of the id
        # type: string
        self.id_name = 'product_id'
        
        # setup services for data manipulation
        self.init_ros()

    def init_ros(self):
        added_param = rospy.get_param(
            '~product_added', PRODUCT_ADDED)
        self.added_pub = rospy.Publisher(
            added_param, Product, queue_size=10)
        changed_param = rospy.get_param(
            '~product_changed', PRODUCT_CHANGED)
        self.changed_pub = rospy.Publisher(
            changed_param, Product, queue_size=10)
        removed_param = rospy.get_param(
            '~product_removed', PRODUCT_REMOVED)
        self.removed_pub = rospy.Publisher(
            removed_param, String, queue_size=10)
        
        create_param = rospy.get_param(
            '~product_create', PRODUCT_CREATE)
        self.create_srv = rospy.Service(
            create_param, ProductCreate, self.on_create)
        delete_param = rospy.get_param(
            '~product_delete', PRODUCT_DELETE)
        self.delete_srv = rospy.Service(
            delete_param, ProductDelete, self.on_delete)
        get_param = rospy.get_param(
            '~product_get', PRODUCT_GET)
        self.get_srv = rospy.Service(
            get_param, ProductGet, self.on_get)
        list_param = rospy.get_param(
            '~product_list', PRODUCT_LIST)
        self.list_srv = rospy.Service(
            list_param, ProductList, self.on_list)
        update_param = rospy.get_param(
            '~product_update', PRODUCT_UPDATE)
        self.update_srv = rospy.Service(
            update_param, ProductUpdate, self.on_update)
        overwrite_param = rospy.get_param(
            '~product_overwrite', PRODUCT_OVERWRITE)
        self.overwrite_srv = rospy.Service(
            overwrite_param, ProductOverwrite, self.on_overwrite)
        


    def on_create(self, msg):
        elem = msg.product
        _product_id = elem.product_id
        print(self.data)
        print(_product_id)
        if _product_id in self.data and not self.allow_overwrite:
            rospy.logerr(
                '[ProductNode] Can not create, product_id ' +
                _product_id + ' already exists.')
            return ProductCreateResponse(success=False)
        self.data[_product_id] = elem
        self.added_pub.publish(elem)
        return ProductCreateResponse(success=True)

    def on_delete(self, msg):
        _product_id = msg.product_id
        if _product_id not in self.data:
            rospy.logerr(
                '[ProductNode] Can not delete, product_id ' +
                _product_id + ' does not exist.')
            return ProductDeleteResponse(success=False)
        del self.data[_product_id]
        self.removed_pub.publish(_product_id)
        return ProductDeleteResponse(success=True)

    def on_get(self, msg):
        if msg.product_id in self.data:
            return ProductGetResponse(product=self.data[msg.product_id])
        else:
            rospy.loginfo(
                '[ProductNode] ' + msg.product_id + ' does not exist!')
            return ProductGetResponse()

    def on_list(self, msg):
        return ProductListResponse(product=self.data.values())

    def on_update(self, msg):
        elem = msg.product
        _product_id = elem.product_id
        if _product_id not in self.data:
            rospy.logerr(
                '[ProductNode] Can not update, product_id ' +
                _product_id + ' does not exist.')
            return ProductUpdateResponse(success=False)
        self.data[_product_id] = elem
        self.changed_pub.publish(elem)
        return ProductUpdateResponse(success=True)

    def on_overwrite(self, msg):
        """allow overwrite product in self.data"""
        self.allow_overwrite = msg.allow_overwrite
        return ProductOverwriteResponse()


    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('product_node', anonymous=True)
    node = ProductNode()
    rospy.loginfo('Product node started')
    node.spin()