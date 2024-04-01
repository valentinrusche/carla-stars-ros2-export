"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""

import math
import os
from pathlib import Path
from typing import Any, List, Union

import orjson
from stars_msgs.msg import StarsWaypoint
from ...util.math_operations import rpy_from_quaternion
from ...stars.json_data_classes import LaneMidpoint, Location, Rotation
from ..async_service_client import AsyncServiceClient


class StarsWaypointClient(AsyncServiceClient):

    def __init__(self, node_name: str, message_type, topic_name: str, callback_group, timeout_sec: float = 10.0) -> None:
        """Requests all waypoints from the waypoint publisher service asynchronously
            and publishes them to the system when they are available"""
        super().__init__(node_name=node_name, message_type=message_type, topic_name=topic_name, callback_group=callback_group, timeout_sec=timeout_sec)
        self.result: Union[StarsWaypoint, None] = self.send_request()

        # waypoints array inside the waypoints result from the service
        self.waypoints = self.result.waypoints.waypoints if self.result is not None else None
        self.lane_midpoints: List[Any] = []
        if self.waypoints is not None:
            self.get_logger().info(message=f"Received {len(self.waypoints)} waypoints for the current map.")

            for waypoint in self.waypoints:
                roll, pitch, yaw = rpy_from_quaternion(q=waypoint.pose.orientation)

                midpoint = LaneMidpoint(
                        distance_to_start=math.sqrt(waypoint.pose.position.x**2 + waypoint.pose.position.y**2),
                        location=Location(x=waypoint.pose.position.x, y=waypoint.pose.position.y, z=waypoint.pose.position.z),
                        rotation=Rotation(pitch=pitch, yaw=yaw, roll=roll),
                        lane_id=waypoint.lane_id,
                        road_id=waypoint.road_id
                    )

                self.lane_midpoints.append(midpoint.to_dict())

            waypoint_dir: Path = Path(os.getenv(key="SIMULATION_MAP_FILE_DIR")) #type: ignore
            waypoint_file: Path = waypoint_dir / "waypoint.json"
            with open(file = waypoint_file, mode = "wb") as f:
                f.write(orjson.dumps(self.lane_midpoints, option=orjson.OPT_INDENT_2))
                self.get_logger().info(message=f"Successfully wrote lane midpoints to file {waypoint_file}.")
