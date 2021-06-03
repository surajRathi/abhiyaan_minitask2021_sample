cmake_minimum_required(VERSION 3.0.2)
project(mpc_simple)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        turtlesim
        geometry_msgs
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(cppad REQUIRED)
pkg_check_modules(ipopt REQUIRED)

catkin_package(
        CATKIN_DEPENDS turtlesim
        DEPENDS cppad ipopt
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/mpc_simple_node.cpp src/MPC.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ipopt
        )

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )