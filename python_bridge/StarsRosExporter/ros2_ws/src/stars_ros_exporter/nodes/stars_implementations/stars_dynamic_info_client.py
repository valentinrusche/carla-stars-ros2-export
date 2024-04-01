"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""

from typing import List, Dict, Union, Callable, Set
from threading import Thread
import os
import time
from pathlib import Path
from datetime import datetime
import orjson
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from rclpy.task import Future
from stars_msgs.msg import StarsActorList, StarsActorState
from ..async_service_client import AsyncServiceClient
from ...util.math_operations import rpy_from_quaternion


class StarsDynamicInfoClient(AsyncServiceClient):

    def __init__(self, node_name: str, message_type, topic_name: str, callback_group,
                 tick_time = 0.05, role_name = "ego_vehicle", file_dir = os.getenv(key = "SIMULATION_STARS_DYNAMIC_FILE_DIR")) -> None:
        """Requests all dynamic data available by polling the data each tick"""
        super().__init__(node_name = node_name, message_type = message_type, topic_name = topic_name, callback_group = callback_group)
        self.dynamic_data_list: List[Dict] = []
        self.is_polling = True
        self.received_actor_callback = False
        self.json_dir: Union[Path, None] = Path(file_dir) if file_dir is not None else None
        if self.json_dir is not None:
            self.json_file: Path = self.json_dir / f"{datetime.now().strftime('%Y_%m_%d_%H_%M_%S')}.json"
        self.role_name: str = role_name
        self.actor_ids: Set[int] = set()
        self.results: List[StarsActorState] = []
        self.tick_time: float = tick_time
        self.save_counter: int = 0
        self.actor_list: List[StarsActorList] = []

        callback: Callable[[StarsActorList], None] = lambda list: self.__handle_actors(actors = list.actors)

        self.create_subscription(
            msg_type = StarsActorList, topic = f"/stars/dynamic/all_vehicle_actors",
            callback = callback,
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.thread = Thread(target=self.__update_thread)

        self.thread.start()

    def __update_thread(self) -> None:
        """
        execution loop for async mode actor discovery
        """
        while self.is_polling:
            if self.received_actor_callback:
                for id in self.actor_ids:
                    self.__send_request_for_id(id = id)
                self.__receive_and_write_dynamic_data()
            time.sleep(0.05)

    def destroy_node(self) -> None:
        self.is_polling = False
        self.__save_data_to_disc()
        super().destroy_node()

    def __handle_actors(self, actors) -> None:
        for actor in actors:
            # only add actors if they are still unknown
            if not self.received_actor_callback:
                self.get_logger().info(message = f"Found Actor(id={actor.id}) via subscription on topic '/stars/dynamic/all_vehicle_actors'.")
                self.actor_ids.add(actor.id)

        self.received_actor_callback = True

    def __send_request_for_id(self, id: int) -> None:
        self.results.append(self.send_request(id = id)) # type: ignore

    def send_request(self, id: int):
        msg = self.message_type.Request()
        msg.id = id
        response: Future = self.client.call_async(request=msg)
        rclpy.spin_until_future_complete(node=self, future=response)
        result = response.result()
        state = result.actor_state if result is not None else None
        return state


    def __receive_and_write_dynamic_data(self) -> None:
        for result in self.results:
            if self.save_counter % 100 == 0:
                if result is not None:
                    self.get_logger().info(message=f"Received 100 ActorStates. Current tick is {str(result.current_tick)}.")
            self.__add_data_to_list(result=result)
        if self.save_counter % 10 == 0:
            self.__save_data_to_disc()
        self.results.clear() # remove duplicates for the next tick
        self.save_counter = self.save_counter + 1

    def __save_data_to_disc(self) -> None:

        if self.json_dir is not None and not self.json_dir.exists():
                self.get_logger().info(message=f"Dir {str(self.json_dir)} does not exist yet. Creating it.")
                self.json_dir.mkdir(parents=True, exist_ok=True)
        if self.json_file is not None and not self.json_file.exists():
            self.get_logger().info(message=f"File {str(self.json_file)} does not exist yet. Creating it.")
            self.json_file.touch()

        with open(file = self.json_file, mode = "wb") as f:
            f.write(orjson.dumps(self.dynamic_data_list, option=orjson.OPT_INDENT_2)) #.decode(encoding = "utf-8") ) # we want human readable data not bytes
            self.get_logger().info(message=f"Newest dynamic data written to disc.")

    def __add_data_to_list(self, result) -> None:
        roll, pitch, yaw = rpy_from_quaternion(q=result.vehicle_status.orientation) if result is not None else (None, None, None)
        dynamic_data: Dict = {"current_tick": float(result.current_tick) if result is not None else None,
            "actor_positions": [
                {"position_on_lane": "",
                "road_id": result.current_waypoint.road_id if result is not None else None,
                "lane_id": result.current_waypoint.lane_id if result is not None else None,
                "actor": {
                    "id": result.vehicle_info.id if result is not None else None,
                    "type": result.vehicle_info.rolename if result is not None else None,
                    "type_id": result.vehicle_info.type if result is not None else None,
                    "ego_vehicle": result.is_ego_vehicle if result is not None else None,
                    "location": {
                        "x": result.current_waypoint.pose.position.x if result is not None else None,
                        "y": result.current_waypoint.pose.position.y if result is not None else None,
                        "z": result.current_waypoint.pose.position.z if result is not None else None
                    },
                    "rotation": {
                        "pitch": pitch,
                        "yaw": yaw,
                        "roll": roll
                    },
                    # "velocity": {
                    #     "x": "",
                    #     "y": "",
                    #     "z": ""
                    # },
                    # "acceleration": {
                    #     "x": "",
                    #     "y": "",
                    #     "z": ""
                    # },
                    # "forward_vector": {
                    #     "x": "",
                    #     "y": "",
                    #     "z": ""
                    # },
                    # "angular_velocity": {
                    #     "x": "",
                    #     "y": "",
                    #     "z": ""
                    # },
                    "geo_location": {
                        "longitude": result.geo_longitude if result is not None else None,
                        "latitude": result.geo_latitude if result is not None else None,
                        "altitude": result.geo_altitude if result is not None else None
                    }
                }}
            ]
        }
        self.dynamic_data_list.append(dynamic_data)

