#!/usr/bin/env python3

"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""

import rclpy
from typing import Union
from stars_msgs.srv import StarsGetActorState
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from .util.helpers import bool_val_of_env_key
from .nodes.stars_implementations.stars_dynamic_info_client import StarsDynamicInfoClient
from .nodes.stars_implementations.stars_static_map_reader import StarsStaticMapReader


def main() -> None:
    """Runs this projects primary nodes in ROS2"""
    rclpy.init()

    # we declare all objects here to use them in the except
    node_executor: Union[MultiThreadedExecutor, None] = None
    stars_dynamic_data_client: Union[StarsDynamicInfoClient, None] = None
    stars_static_map_reader: Union[StarsStaticMapReader, None] = None

    callback_group = ReentrantCallbackGroup() # allows for the concurrent execution of nodes

    try:
        node_executor: Union[MultiThreadedExecutor,None] = MultiThreadedExecutor(num_threads=5) # needed to allow multiple nodes to be run inside ROS2

        if bool_val_of_env_key(env_key="ENABLED_FLAG_STATIC_DATA_LOGGING", default="True"):
            stars_static_map_reader = StarsStaticMapReader(node_name = 'Stars_Static_Map_Reader', polling_rate = 10,
                                                                                callback_group = callback_group)
            node_executor.add_node(node=stars_static_map_reader)

        if bool_val_of_env_key(env_key="ENABLED_FLAG_DYNAMIC_DATA_LOGGING", default="True"):
            stars_dynamic_data_client = StarsDynamicInfoClient(node_name = 'Stars_Dynamic_Data_Client', message_type = StarsGetActorState,
                                                                    topic_name = '/stars/dynamic/get_actor_state',
                                                                    callback_group = callback_group)
            node_executor.add_node(node=stars_dynamic_data_client)

        node_executor.spin() # Run all added node callbacks until the program terminates
    except (SystemExit, KeyboardInterrupt):
        if bool_val_of_env_key(env_key="ENABLED_FLAG_STATIC_DATA_LOGGING", default="True") and stars_static_map_reader is not None:
            stars_static_map_reader.destroy_node()
        if bool_val_of_env_key(env_key="ENABLED_FLAG_DYNAMIC_DATA_LOGGING", default="True") and stars_dynamic_data_client is not None:
            stars_dynamic_data_client.destroy_node()
    finally:
        rclpy.shutdown() # shutdown ROS2

        if node_executor is not None:
            node_executor.shutdown()

if __name__ == "__main__":
    main()
