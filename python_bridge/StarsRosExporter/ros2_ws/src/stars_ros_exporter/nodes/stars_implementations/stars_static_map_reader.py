"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""

import os
from typing import Callable
from pathlib import Path
from threading import Thread
import time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from rclpy.impl.rcutils_logger import RcutilsLogger
from stars_msgs.msg import StarsWorldInfo
from stars_msgs.srv import StarsGetAllWaypoints
from ...stars import dataclass_to_json_converter
from .stars_waypoint_client import StarsWaypointClient


class StarsStaticMapReader(Node):

    def __init__(self, node_name: str, polling_rate: int, callback_group) -> None:
        """Creates a ROS2 topic subscription listening for the current map data and calling _write_static_data_to_file
            to write it to disk"""
        super().__init__(node_name=node_name, parameter_overrides=[])
        self.polling_rate: int = polling_rate

        self.is_exporting_done = False
        self.received_world_info = False

        callback: Callable[[StarsWorldInfo], None] = lambda world_info: self.__save_world_info(world_info=world_info)
        self.create_subscription(
            msg_type=StarsWorldInfo, topic="/stars/static/world_info",
            callback=callback,
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.waypoint_client: StarsWaypointClient = StarsWaypointClient(node_name = 'Stars_Waypoint_Client', message_type = StarsGetAllWaypoints,
                                                                    topic_name = '/stars/static/waypoints/get_all_waypoints',
                                                                    callback_group = callback_group, timeout_sec = 10.0)

        self.thread = Thread(target=self.__update_thread)

        self.thread.start()

        self.get_logger().info(message=f"Successfully created. Starting processing of static map data.")

    def __save_world_info(self, world_info: StarsWorldInfo) -> None:
        self.get_logger().info(message="Received newest world info.")
        self.world_info: StarsWorldInfo = world_info
        self.received_world_info = True

    def __update_thread(self) -> None:
        """
        execution loop for async mode actor discovery
        """
        while not self.is_exporting_done:
            if self.received_world_info and self.waypoint_client.waypoints is not None:
                self.__write_static_data_to_file(map_name=Path(self.world_info.map_name), 
                                                 map_data=self.world_info.map_data, map_format=self.world_info.map_format, logger = self.get_logger())
            time.sleep(self.polling_rate)

        self.get_logger().info(message="Done exporting static map data.")
        # Work was finished so we can stop the node from spinning infinitely
        self.destroy_node()

    def __write_static_data_to_file(self, map_name: Path, map_data: str, map_format: str, logger: RcutilsLogger) -> None:
        """Creates a map file containing the read opendrive data xml string creating the desired path if it
        not yet exists. xml_dir is the name of the OpenDrive map dir under the SIMULATION_MAP_FILE_DIR path. 
        json_dir ist the name of the STARS compatible JSON file under the SIMULATION_STARS_STATIC_FILE_DIR path.
        """

        stars_json_dir: Path = Path(os.getenv(key="SIMULATION_STARS_STATIC_FILE_DIR")) / map_name.parent # type: ignore
        stars_json_file: Path = stars_json_dir / f"{str(map_name.name)}.json"

        if map_format == "xodr":
            self.__save_xodr_data(map_name=map_name, data=map_data)

            self.__save_as_stars_json(map_name=map_name, data=map_data, json_dir=stars_json_dir, json_file=stars_json_file)
        else:
            raise NotImplementedError(f"Map format {map_format} is not supported yet.")
        self.is_exporting_done = True

    def __save_as_stars_json(self, map_name: Path, data: str, json_dir: Path, json_file: Path) -> None: 
        dataclass_to_json_converter.export_to_json(map_name=map_name, data=data, json_dir=json_dir, json_file=json_file, logger = self.get_logger(), waypoints=self.waypoint_client.waypoints)

    def __save_xodr_data(self, map_name: Path, data: str) -> None:

        xodr_dir: Path = Path(os.getenv(key="SIMULATION_MAP_FILE_DIR")) / map_name.parent # type: ignore
        xodr_file: Path = xodr_dir / f"{str(map_name.name)}.xodr"

        if not xodr_dir.exists():
            self.get_logger().info(message=f"Dir {str(xodr_dir)} does not exist yet. Creating it.")
            xodr_dir.mkdir(parents=True, exist_ok=True)

        if not xodr_file.exists():
            self.get_logger().info(message=f"File {str(xodr_file)} does not exist yet. Creating it.")
            xodr_file.touch()
            with open(file=xodr_file, mode="w") as f:
                f.write(data)
            self.get_logger().info(message=f"Successfully saved map {str(map_name.name)} to {str(xodr_file)} in OpenDrive format.")

