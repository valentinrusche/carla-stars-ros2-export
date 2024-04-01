"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""

from math import atan2, asin
import itertools
from typing import Dict, Tuple

import numpy as np
from shapely.geometry import LineString, Point


def rpy_from_quaternion(q) -> Tuple[float, float, float]:
    roll: float = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
    roll = abs(roll) if roll == -0.0 else roll
    pitch: float = asin(-2.0*(q.x*q.z - q.w*q.y))
    pitch = abs(pitch) if pitch == -0.0 else pitch
    yaw: float = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    yaw = abs(yaw) if yaw == -0.0 else yaw
    yaw += 90 # carla uses offset of 90 degrees
    return roll, pitch, yaw

def create_linestring_from_lane(lane_start_x: float, lane_start_y: float, lane_heading: float, lane_length: float) -> LineString:
    # Get the start and end points of the line
    start_point: Tuple[float, float] = (lane_start_x, lane_start_y)
    end_point: Tuple[float, float] = (lane_start_x + lane_length * np.cos(lane_heading), lane_start_y + lane_length * np.sin(lane_heading))

    # Create a LineString from the points
    return LineString([start_point, end_point])

def cross_check_intersections(lanes_dict: Dict[str, LineString], current_id: str) -> Dict[str, Point]:
    keys = list(lanes_dict.keys())
    intersections: Dict[str, Point] = {}
    for perm in itertools.permutations(iterable=keys, r=2):
        if not f"{perm[0]}+{perm[1]}" in intersections.keys() and\
        not f"{perm[1]}+{perm[0]}" in intersections.keys()\
        and (perm[0].startswith(current_id) or perm[1].startswith(current_id)):
            intersection = lanes_dict[perm[0]].intersection(lanes_dict[perm[1]])
            if not intersection.is_empty and isinstance(intersection, Point):
                intersections[f"{perm[0]}+{perm[1]}"] = intersection
    return intersections