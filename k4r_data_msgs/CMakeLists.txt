cmake_minimum_required(VERSION 2.8.3)
project(k4r_data_msgs)

find_package(catkin REQUIRED message_generation geometry_msgs std_msgs)

add_service_files(FILES ShelfCreate.srv ShelfList.srv ShelfUpdate.srv
  ShelfDelete.srv ProductCreate.srv ProductList.srv ProductUpdate.srv
  ProductDelete.srv ShelfGet.srv ProductGet.srv ShelfOverwrite.srv
  ProductOverwrite.srv)

add_message_files(FILES Shelf.msg Product.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime rospy)
