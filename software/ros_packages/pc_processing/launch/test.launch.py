#!/usr/env/bin python3

#Launch file for hw5, opens the relevant nodes and rviz config. 

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
	
	return LaunchDescription([
		#Launch the pc_filter node
		Node(
			package = 'pc_processing',
			executable = 'pc_filter',
			name = 'pc_filter',
			remappings = [
				('/raw_point_cloud', '/astra_ros/devices/default/point_cloud')
			]
		),
		#Launch the person detector node
		Node(
			package = 'pc_processing',
			executable = 'plane_fit',
			name = 'plane_fit',
		),
		#Launch the personal space node
		Node(
			package = 'pc_processing',
			executable = 'count_objects',
			name = 'count_objects'
		),
		#Launch Rviz2:
		Node(
			package = 'rviz2',
			executable = 'rviz2',
			name = 'rviz2',
			arguments = ['-d' +  os.path.join(get_package_share_directory('hw7'),'config/bartender.rviz')]
		)

	])
