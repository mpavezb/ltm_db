cmake_minimum_required(VERSION 2.8.3)
project(ltm_db)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pluginlib
  roscpp
  rostest
  rostime
  std_msgs
  tf
)
find_package(Boost COMPONENTS system filesystem thread)
find_package(OpenSSL)
find_package(MongoDB)

set(MONGO_EXPORT)
if("${MongoDB_LIBRARIES}" MATCHES "\\.so$")
  set(MONGO_EXPORT MongoDB)
endif()

file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")


###################################
## catkin specific configuration ##
###################################
catkin_python_setup()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ltm_db_interface ltm_db_mongo
  CATKIN_DEPENDS roscpp rostime std_msgs geometry_msgs
  DEPENDS Boost
)


###########
## Build ##
###########
if (NOT MongoDB_EXPOSE_MACROS)
  add_definitions(-DMONGO_EXPOSE_MACROS)
endif()

configure_file(
  "include/ltm_db/mongo/config.h.in"
  "${CATKIN_DEVEL_PREFIX}/include/ltm_db/mongo/config.h"
)

include_directories(
  ${CATKIN_DEVEL_PREFIX}/include
  include
  ${catkin_INCLUDE_DIRS}
  ${MongoDB_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)
link_directories(${catkin_LINK_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(ltm_db_interface_srcs
  src/interface/database_loader.cpp
  src/interface/transform_collection.cpp)

add_library(ltm_db_interface SHARED ${ltm_db_interface_srcs})
target_link_libraries(ltm_db_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ltm_db_test_dbloader src/interface/test_dbloader.cpp)
target_link_libraries(ltm_db_test_dbloader ltm_db_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})


set(ltm_db_mongo_srcs
  src/mongo/util.cpp
  src/mongo/mongo_dummy.cpp
  src/mongo/query_results.cpp
  src/mongo/message_collection.cpp
  src/mongo/database_connection.cpp)

add_library(ltm_db_mongo SHARED ${ltm_db_mongo_srcs})
target_link_libraries(ltm_db_mongo ${catkin_LIBRARIES} ${MongoDB_LIBRARIES} ${OPENSSL_LIBRARIES} ${Boost_LIBRARIES})


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_ltm_db_mongo_cpp test/mongo/test_mongo.cpp)
  target_link_libraries(test_ltm_db_mongo_cpp ltm_db_mongo)
endif()

#############
## Install ##
#############
install(TARGETS ltm_db_interface LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

install(PROGRAMS src/mongo/mongo_wrapper_ros.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ltm_db_mongo LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ltm_db/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(FILES "${CATKIN_DEVEL_PREFIX}/include/ltm_db/mongo/config.h"
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES mongo_database_connection_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
