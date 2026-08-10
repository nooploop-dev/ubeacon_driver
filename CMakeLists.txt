cmake_minimum_required(VERSION 3.16)

set(PKG_NAME ubeacon_driver)

project(${PKG_NAME})

# 未显式选择ROS版本时，尝试根据环境变量自动检测
set(_ub_ros1_default OFF)
set(_ub_ros2_default OFF)
if(NOT DEFINED UB_BUILD_ROS1 AND NOT DEFINED UB_BUILD_ROS2)
    if("$ENV{ROS_VERSION}" STREQUAL "1")
        set(_ub_ros1_default ON)
    elseif("$ENV{ROS_VERSION}" STREQUAL "2")
        set(_ub_ros2_default ON)
    endif()
endif()
option(UB_BUILD_ROS1 "build ROS1 converter" ${_ub_ros1_default})
option(UB_BUILD_ROS2 "build ROS2 converter" ${_ub_ros2_default})

# 非ROS构建时默认启用reader/writer/test以及FOXGLOVE可视化支持， ROS构建时这些默认关闭
if(UB_BUILD_ROS1 OR UB_BUILD_ROS2)
    set(_ub_native_default OFF)
else()
    set(_ub_native_default ON)
endif()
option(UB_BUILD_READER "build reader" ${_ub_native_default})
option(UB_BUILD_WRITER "build writer" ${_ub_native_default})
option(UB_BUILD_TEST "build test" ${_ub_native_default})
option(
    UB_BUILD_FOXGLOVE
    "build reader with foxglove visualization"
    ${_ub_native_default}
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
foreach(lang C CXX)
    foreach(config RELEASE MINSIZEREL RELWITHDEBINFO)
        string(
            REGEX REPLACE "[-/]D[ ]?NDEBUG"
            ""
            CMAKE_${lang}_FLAGS_${config}
            "${CMAKE_${lang}_FLAGS_${config}}"
        )
    endforeach()
endforeach()
if(MSVC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /utf-8")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8")
    add_compile_options(/W4)
    # foxglove sdk 需要禁用MSVC的permissive-模式，否则编译报错
    # add_compile_options(/permissive-)
    add_compile_definitions(NOMINMAX)
    add_compile_definitions(_WIN32_WINNT=0x0A00)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(LIB_DEV_DRIVER lib_${PKG_NAME})
add_library(${LIB_DEV_DRIVER} STATIC src/ubeacon_frame.c src/ubeacon_driver.c)
target_include_directories(${LIB_DEV_DRIVER} PUBLIC include)
if(TARGET ${LIB_DEV_DRIVER}_extend)
    target_link_libraries(${LIB_DEV_DRIVER} PUBLIC ${LIB_DEV_DRIVER}_extend)
endif()

include(FetchContent)
if(UB_BUILD_TEST)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.7.1
    )
    FetchContent_MakeAvailable(Catch2)
    enable_testing()
    add_subdirectory(test)
endif()

if(UB_BUILD_ROS1)
    find_package(
        catkin
        REQUIRED
        COMPONENTS roscpp std_msgs geometry_msgs message_generation
    )
    add_message_files(
            FILES
            AnchorDdoa.msg
            AnchorDdoas.msg
            AnchorPos.msg
            AnchorSignal.msg
            AnchorSignalData.msg
            BleInterfaceParam.msg
            Find.msg
            GlobalTimeStatus.msg
            Heartbeat.msg
            IicInterfaceParam.msg
            InterfaceParam.msg
            LocationAnchor.msg
            LocationResult.msg
            MapMeasurement.msg
            Param.msg
            Restart.msg
            RunTimeParam.msg
            StateControl.msg
            UartInterfaceParam.msg
            UserData.msg
            UwbInterfaceParam.msg
            ZMeasurement.msg
    )
    generate_messages(DEPENDENCIES std_msgs)
    catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime)
endif()

if(UB_BUILD_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(
            ${PKG_NAME}
            "msg/AnchorDdoa.msg"
            "msg/AnchorDdoas.msg"
            "msg/AnchorPos.msg"
            "msg/AnchorSignal.msg"
            "msg/AnchorSignalData.msg"
            "msg/BleInterfaceParam.msg"
            "msg/Find.msg"
            "msg/GlobalTimeStatus.msg"
            "msg/Heartbeat.msg"
            "msg/IicInterfaceParam.msg"
            "msg/InterfaceParam.msg"
            "msg/LocationAnchor.msg"
            "msg/LocationResult.msg"
            "msg/MapMeasurement.msg"
            "msg/Param.msg"
            "msg/Restart.msg"
            "msg/RunTimeParam.msg"
            "msg/StateControl.msg"
            "msg/UartInterfaceParam.msg"
            "msg/UserData.msg"
            "msg/UwbInterfaceParam.msg"
            "msg/ZMeasurement.msg"
            DEPENDENCIES
            std_msgs
    )
    ament_export_dependencies(rosidl_default_runtime)
    install(DIRECTORY launch rviz DESTINATION share/${PKG_NAME})
endif()

add_subdirectory(app)

if(UB_BUILD_ROS2)
    ament_package()
endif()
