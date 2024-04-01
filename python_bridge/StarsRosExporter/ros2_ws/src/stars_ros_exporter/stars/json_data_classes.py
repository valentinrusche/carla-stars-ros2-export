"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""

from dataclasses import dataclass, field
from typing import List
from enum import IntEnum


class LaneType(IntEnum):
    Any = -2
    Bidirectional = 512
    Biking = 16
    Border = 64
    Driving = 2
    Entry = 131072
    Exit = 262144
    Median = 1024
    NONE = 1
    OffRamp = 524288
    OnRamp = 1048576
    Parking = 256
    Rail = 65536
    Restricted = 128
    RoadWorks = 16384
    Shoulder = 8
    Sidewalk = 32
    Special1 = 2048
    Special2 = 4096
    Special3 = 8192
    Stop = 4
    Tram = 32768

@dataclass
class Location:
    x: float
    y: float
    z: float

    def __repr__(self) -> str:
        return f"({self.x},{self.y},{self.z})"

    def __str__(self) -> str:
        return f"({self.x},{self.y},{self.z})"

@dataclass
class ContactArea:
    id: str
    contact_location: Location
    lane_1_road_id: int
    lane_1_id: int
    lane_1_start_pos: float
    lane_1_end_pos: float
    lane_2_road_id: int
    lane_2_id: int
    lane_2_start_pos: float
    lane_2_end_pos: float

@dataclass
class Rotation:
    pitch: float
    yaw: float
    roll: float

@dataclass
class LaneMidpoint:
    lane_id: int
    road_id: int
    distance_to_start: float
    location: Location
    rotation: Rotation

    def to_dict(self):
        ret = \
            {
                'lane_id': self.lane_id, 
                'road_id': self.road_id,
                'distance_to_start': self.distance_to_start,
                'location': {'x': self.location.x, 'y': self.location.y, 'z': self.location.z},
                'rotation': {'pitch': self.rotation.pitch, 'yaw': self.rotation.yaw, 'roll': self.rotation.roll}
            }
        return ret

@dataclass
class ContactLaneInfo:
    road_id: int
    lane_id: int

@dataclass
class SpeedLimit:
    speed_limit: float
    from_distance_from_start: float
    to_distance_from_start: float

class LandmarkOrientation(IntEnum):
    Positive = 0
    Negative = 1
    Both = 2

class LandmarkType(IntEnum):
    Danger = 101
    Lanes_Merging = 121
    Caution_Pedestrian = 133
    Caution_Bicycle = 138
    Level_Crossing = 150
    Stop_Sign = 206
    Yield_Sign = 205
    Mandatory_Turn_Direction = 209
    Mandatory_Left_Right_Direction = 211
    Two_Choice_Turn_Direction = 214
    Roundabout = 215
    Pass_Right_Left = 222
    Access_Forbidden = 250
    Access_Forbidden_Motorvehicles = 251
    Access_Forbidden_Trucks = 253
    Access_Forbidden_Bicycle = 254
    Access_Forbidden_Weight = 263
    Access_Forbidden_Width = 264
    Access_Forbidden_Height = 265
    Access_Forbidden_Wrong_Direction = 267
    Forbidden_U_Turn = 272
    Maximum_Speed = 274
    Forbidden_Overtaking_Motorvehicles = 276
    Forbidden_Overtaking_Trucks = 277
    Absolute_No_Stop = 283
    Restricted_Stop = 286
    Has_Way_Next_Intersection = 301
    Priority_Way = 306
    Priority_Way_End = 307
    City_Begin = 310
    City_End = 311
    Highway = 330
    Dead_End = 357
    Recommended_Speed_End = 381
    Light_Post = 1000001

@dataclass
class Landmark:
    id: int
    road_id: int
    name: str
    distance: float
    s: float
    is_dynamic: bool
    orientation: LandmarkOrientation
    z_offset: float
    country: str
    type: LandmarkType
    sub_type: str
    value: float
    unit: str
    height: float
    width: float
    text: str
    h_offset: float
    pitch: float
    roll: float
    location: Location
    rotation: Rotation

@dataclass
class StaticTrafficLight:
    # id: int Not needed anymore?
    location: Location
    rotation: Rotation
    open_drive_id: str
    position_distance: float
    stop_locations: List[Location] = field(default_factory=list)

@dataclass
class Lane:
    road_id: int
    lane_id: int
    lane_type: LaneType
    lane_width: float
    lane_length: float
    s: float
    predecessor_lanes: List[ContactLaneInfo] = field(default_factory=list)
    successor_lanes: List[ContactLaneInfo] = field(default_factory=list)
    intersecting_lanes: List[ContactLaneInfo] = field(default_factory=list)
    lane_mid_points: List[LaneMidpoint] = field(default_factory=list)
    speed_limits: List[SpeedLimit] = field(default_factory=list)
    landmarks: List[Landmark] = field(default_factory=list)
    contact_areas: List[ContactArea] = field(default_factory=list)
    traffic_lights: List[StaticTrafficLight] = field(default_factory=list)

@dataclass
class Road:
    road_id: int
    is_junction: bool
    lanes: List[Lane] = field(default_factory = list)

@dataclass
class Block:
    id: str
    roads: List[Road] = field(default_factory = list)
