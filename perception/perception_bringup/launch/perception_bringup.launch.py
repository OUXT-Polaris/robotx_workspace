# Copyright (c) 2020 OUXT Polaris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os

import ament_index_python.packages

import yaml


def generate_launch_description():
    container = ComposableNodeContainer(
            node_name='preception_bringup_container',
            node_namespace='perception',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                # getImageDecompressorComponent('front_camera'),
                # getImageRectifyComponent('front_camera'),
                getScanSgementationComponent(),
                getCropHullFilterComponent(),
                getPointCloudToLaserScanComponent(),
                getRadiusOutlierRemovalComponent('front_lidar'),
                getRadiusOutlierRemovalComponent('rear_lidar'),
                getRadiusOutlierRemovalComponent('right_lidar'),
                getRadiusOutlierRemovalComponent('left_lidar'),
                getPointsTransformComponent('front_lidar'),
                getPointsTransformComponent('rear_lidar'),
                getPointsTransformComponent('right_lidar'),
                getPointsTransformComponent('left_lidar'),
                getPointsConcatenateComponent()
            ],
            output='screen',
    )
    return launch.LaunchDescription([container])


def getPointsTransformComponent(lidar_name):
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('perception_bringup'),
        'config')
    param_config = os.path.join(config_directory, lidar_name+'_points_transform.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)[lidar_name + '_points_transform_node']['ros__parameters']
    component = ComposableNode(
        package='pcl_apps',
        plugin='pcl_apps::PointsTransformComponent',
        namespace='/perception/'+lidar_name,
        name='points_transform_node',
        remappings=[("input", lidar_name+"/points_raw"), ("output", "points_raw/transformed")],
        parameters=[params])
    return component


def getScanSgementationComponent():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('perception_bringup'),
        'config')
    param_config = os.path.join(config_directory, 'scan_segmentation.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['scan_segmentation_node']['ros__parameters']
    component = ComposableNode(
        package='scan_segmentation',
        plugin='scan_segmentation::ScanSegmentationComponent',
        namespace='/perception',
        name='scan_segmentation_node',
        parameters=[params])
    return component


def getCropHullFilterComponent():
    component = ComposableNode(
        package='pcl_apps',
        plugin='pcl_apps::CropHullFilterComponent',
        namespace='/perception',
        remappings=[
            ("points", "points_concatenate_node/output"),
            ("polygon", "scan_segmentation_node/polygon")
        ],
        name='crop_hull_filter_node')
    return component


def getPointsConcatenateComponent():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('perception_bringup'),
        'config')
    param_config = os.path.join(config_directory, 'points_concatenate.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['points_concatenate_node']['ros__parameters']
    component = ComposableNode(
        package='pcl_apps',
        plugin='pcl_apps::PointsConcatenateComponent',
        namespace='perception',
        name='points_concatenate_node',
        parameters=[params])
    return component


def getCropBoxFilterComponent(lidar_name):
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('perception_bringup'),
        'config')
    param_config = os.path.join(config_directory, 'crop_box_filter.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['crop_box_filter_node']['ros__parameters']
    component = ComposableNode(
        package='pcl_apps',
        plugin='pcl_apps::CropBoxFilterComponent',
        namespace='/perception/'+lidar_name,
        name='crop_box_filter_node',
        remappings=[("points", "points_raw/transformed")],
        parameters=[params])
    return component


def getPointCloudToLaserScanComponent():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('perception_bringup'),
        'config')
    param_config = os.path.join(config_directory, 'pointcloud_to_laserscan.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['pointcloud_to_laserscan_node']['ros__parameters']
    component = ComposableNode(
        package='pcl_apps',
        plugin='pcl_apps::PointCloudToLaserScanComponent',
        namespace='/perception/',
        name='pointcloud_to_laserscan_node',
        remappings=[
            ("input", "points_concatenate_node/output"),
            ("output", "pointcloud_to_laserscan_node/output")
        ],
        parameters=[params])
    return component


def getRadiusOutlierRemovalComponent(lidar_name):
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('perception_bringup'),
        'config')
    param_config = os.path.join(config_directory, lidar_name+'_radius_outlier_removal.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)[lidar_name + '_radius_outlier_removal_node']['ros__parameters']
    component = ComposableNode(
        package='pcl_apps',
        plugin='pcl_apps::RadiusOutlierRemovalComponent',
        namespace='/perception/'+lidar_name,
        name='radius_outlier_removal_node',
        parameters=[params])
    return component


def getImageDecompressorComponent(camera_name):
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('perception_bringup'),
        'config')
    param_config = os.path.join(config_directory, camera_name+'_image_decompressor.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)[camera_name+'_image_decompressor_node']['ros__parameters']
    component = ComposableNode(
        package='image_processing_utils',
        plugin='image_processing_utils::ImageDecompressorComponent',
        namespace='/perception/'+camera_name,
        name='image_decompressor_node',
        parameters=[params])
    return component


def getImageRectifyComponent(camera_name):
    component = ComposableNode(
        package='image_processing_utils',
        plugin='image_processing_utils::ImageRectifyComponent',
        namespace='/perception/'+camera_name,
        name='image_rectify_node',
        remappings=[("image", "image_raw")],
        parameters=[])
    return component
