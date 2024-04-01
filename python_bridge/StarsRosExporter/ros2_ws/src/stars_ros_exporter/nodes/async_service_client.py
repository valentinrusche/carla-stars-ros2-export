"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future


class AsyncServiceClient(Node):

    def __init__(self, node_name: str, message_type, topic_name: str, callback_group, timeout_sec: float = 10.0) -> None:
        # Create a client
        super().__init__(node_name=node_name, parameter_overrides=[])
        self.timeout: float = timeout_sec
        self.client: Client = self.create_client(srv_type=message_type, srv_name=topic_name, callback_group=callback_group)

        # Check if the a service is available
        while not self.client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().info(message="Waiting for available service.")

        self.message_type = message_type

        self.request = self.message_type.Request()

    def send_request(self):
        self.response: Future = self.client.call_async(request=self.request)
        rclpy.spin_until_future_complete(node=self, future=self.response)
        return self.response.result()
