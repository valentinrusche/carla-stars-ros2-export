"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""

import json
import math
import os
from itertools import product

from dataclasses import replace

from pathlib import Path
from time import sleep
from typing import Dict, List, Set, Tuple, Union

from shapely.geometry.base import BaseGeometry
from shapely.geometry import LineString, MultiPoint, Point

from ..util.errors import XODRError
from rclpy.impl.rcutils_logger import RcutilsLogger
import orjson
from .json_data_classes import (
    Block,
    ContactArea,
    ContactLaneInfo,
    Landmark,
    LandmarkOrientation,
    LandmarkType,
    Lane,
    LaneMidpoint,
    LaneType,
    Location,
    Road,
    Rotation,
    SpeedLimit,
    StaticTrafficLight
)
from ..xodr import xml_to_dataclass_converter
from ..xodr.xodr_dataclasses import (
    XODRJunction,
    XODRRoad,
    XODRRoadLaneSection,
    XODRRoadLaneSectionLCRLane,
    XODRRoadObjectsElement,
    XODRRoadSignalReference
)


def export_to_json(map_name: Path, data: str, json_dir: Path, json_file: Path, logger: RcutilsLogger, waypoints) -> None:
    if not json_dir.exists():
        logger.info(message = f"Dir {str(json_dir)} does not exist yet. Creating it.")
        json_dir.mkdir(parents = True, exist_ok = True)
    if not json_file.exists():
        logger.info(message = f"File {str(json_file)} does not exist yet. Creating it.")
        json_file.touch()

    open_drive_data, filtered_roads, removed_roads = xml_to_dataclass_converter.map_xml_to_dataclass(xodr_text=data, logger=logger) # type: ignore

    if open_drive_data is not None and filtered_roads is not None and removed_roads is not None:

        waypoints_in_junction = [waypoint for waypoint in waypoints if waypoint.is_junction]
        waypoints_in_junction.sort(key=lambda waypoint: (waypoint.road_id, waypoint.lane_id))
        # grouped_waypoints = [list(group) for _, group in groupby(waypoints_in_junction, key=lambda waypoint: (waypoint.road_id, waypoint.lane_id))]

        # for group in grouped_waypoints:
        for wp in waypoints_in_junction:
            logger.debug(f"{wp.road_id}:{wp.lane_id}, {wp.is_junction=}, {wp.pose.position.x=}, {wp.pose.position.y=}, {wp.pose.position.z}")

        block_list: List[Block] = __build_stars_road_blocks(xodr_roads=filtered_roads, removed_road_ids=removed_roads, junctions=open_drive_data.junctions, logger=logger)
        with open(file = json_file, mode = "wb") as f:
            f.write(orjson.dumps(block_list, option=orjson.OPT_INDENT_2))

        # We have to replace lane_mid_points because the kotlin deserializer expects lane_midpoints and the python parts adds the extra underscore
        file_content: str = ""
        with open(file = json_file, mode = "r") as f:
            file_content: str = f.read()
        file_content = file_content.replace("lane_mid_points", "lane_midpoints") #type: ignore
        with open(file = json_file, mode = "w") as f:
            f.write(file_content)

        logger.info(message=f"Successfully wrote road blocks to file {json_file}.")

def __generate_dict_of_pred_succ_lanes(road: XODRRoad) -> Tuple[List[ContactLaneInfo], List[ContactLaneInfo]]:
    predecessor_lanes: List[ContactLaneInfo] = []
    successor_lanes: List[ContactLaneInfo] = []
    if road.link.predecessor is not None and road.lanes.laneSection.center is not None:
        for lane in road.lanes.laneSection.center.lanes:
            if lane.link.predecessor is not None:
                predecessor_lanes.append(
                    ContactLaneInfo(
                        road_id=road.link.predecessor.elementId,
                        lane_id=lane.link.predecessor.id
                    )
                )
    if road.link.predecessor is not None and road.lanes.laneSection.left is not None:
        for lane in road.lanes.laneSection.left.lanes:
            if lane.link.predecessor is not None:
                predecessor_lanes.append(
                    ContactLaneInfo(
                        road_id=road.link.predecessor.elementId,
                        lane_id=lane.link.predecessor.id
                    )
                )
    if road.link.predecessor is not None and road.lanes.laneSection.right is not None:
        for lane in road.lanes.laneSection.right.lanes:
            if lane.link.predecessor is not None:
                predecessor_lanes.append(
                    ContactLaneInfo(
                        road_id=road.link.predecessor.elementId,
                        lane_id=lane.link.predecessor.id
                    )
                )

    if road.link.successor is not None and road.lanes.laneSection.center is not None:
        for lane in road.lanes.laneSection.center.lanes:
            if lane.link.successor is not None:
                successor_lanes.append(
                    ContactLaneInfo(
                        road_id=road.link.successor.elementId,
                        lane_id=lane.link.successor.id
                    )
                )
    if road.link.successor is not None and road.lanes.laneSection.left is not None:
        for lane in road.lanes.laneSection.left.lanes:
            if lane.link.successor is not None:
                successor_lanes.append(
                    ContactLaneInfo(
                        road_id=road.link.successor.elementId,
                        lane_id=lane.link.successor.id
                    )
                )
    if road.link.successor is not None and road.lanes.laneSection.right is not None:
        for lane in road.lanes.laneSection.right.lanes:
            if lane.link.successor is not None:
                successor_lanes.append(
                    ContactLaneInfo(
                        road_id=road.link.successor.elementId,
                        lane_id=lane.link.successor.id
                    )
                )
    return predecessor_lanes, successor_lanes

def __build_contact_areas(conctact_point_dict: Dict[int, Dict[int, Tuple[int, int, BaseGeometry]]], current_road: int, current_lane: int, lane_midpoints: List[LaneMidpoint], logger: RcutilsLogger) -> List[ContactArea]:
    contact_areas: List[ContactArea] = []

    filtered_lane_midpoints: List[LaneMidpoint] = [lane_midpoint for lane_midpoint in lane_midpoints if lane_midpoint.lane_id == current_lane and lane_midpoint.road_id == current_road]

    def find_minimum_distance(points: List[LaneMidpoint], fixed_point: Tuple[float, float]) -> float: 
        return min(math.sqrt((point.location.x - fixed_point[0])**2 + (point.location.y - fixed_point[1])**2) for point in points)

    for road_id, lanes in conctact_point_dict.items():
        for lane_id, tuple_vals in lanes.items():
            if road_id == current_road and lane_id == current_lane:
                other_road_id: int= tuple_vals[0]
                other_lane_id: int= tuple_vals[1]
                geom: BaseGeometry = tuple_vals[2]
                logger.debug(message=str(geom))
                x_pos: float = 0.0
                y_pos: float = 0.0
                if isinstance(geom, LineString):
                    x_pos, y_pos = geom.coords[0]
                elif isinstance(geom, MultiPoint):
                    geom = geom.geoms[0]
                    x_pos = geom.x
                    y_pos = geom.y
                elif isinstance(geom, Point):
                    x_pos = geom.x
                    y_pos = geom.y
                else: # GeometryCollection
                    for elem in geom.geoms:
                        if isinstance(elem, LineString):
                            x_pos, y_pos = elem.coords[0]
                            break
                        elif isinstance(elem, MultiPoint):
                            geom = geom[0]
                            x_pos = geom.x
                            y_pos = geom.y
                            break
                        else: # usual Point class then..
                            x_pos = geom.x
                            y_pos = geom.y
                            break
                min_dist: float = find_minimum_distance(points=filtered_lane_midpoints, fixed_point=(x_pos, y_pos))
                contact_area: ContactArea = ContactArea(
                    id=f"{road_id}_{lane_id}+{other_road_id}_{other_lane_id}",
                    contact_location=Location(x=float(x_pos),y=float(y_pos),z=0.0),
                    lane_1_road_id=road_id,
                    lane_1_id=lane_id,
                    lane_1_start_pos=0.0, #always 0.0 in other dataset
                    lane_1_end_pos=min_dist, #always between 3.1 and 3.7 in other dataset?
                    lane_2_road_id=other_road_id,
                    lane_2_id=other_lane_id,
                    lane_2_start_pos=0.0, #always 0.0 in other dataset
                    lane_2_end_pos=min_dist, #always between 3.1 and 3.7 in other dataset?
                )
                contact_areas.append(contact_area)

    return contact_areas

def __build_lane_midpoints(lane: XODRRoadLaneSectionLCRLane, road: XODRRoad, logger: RcutilsLogger) -> List[LaneMidpoint]:
    lane_mid_points: List[LaneMidpoint] = []
    waypoint_dir: Path = Path(os.getenv(key="MAP_FILE_DIR")) #type: ignore
    waypoint_file: Path = waypoint_dir / "waypoint.json"
    while not waypoint_file.exists():
        sleep(5)
        logger.info(message="Waiting for waypoint file to be created...")
    with open(file=waypoint_file, mode='r') as data_file:
        logger.debug(message=f"Waypoint file {waypoint_file} available, reading it for lane {lane.id} of road {road.id}...")
        json_obj: str = data_file.read()

        data = json.loads(json_obj)
        for elem in data: # should be deserialized to a list of dicts
            if isinstance(elem, dict) and "road_id" in elem and int(elem["road_id"]) == road.id:
                lane_mid_points.append(
                    LaneMidpoint(
                        road_id=elem["road_id"],
                        lane_id=elem["lane_id"],
                        distance_to_start=elem["distance_to_start"],
                        location=Location(
                            x=elem["location"]["x"],
                            y=elem["location"]["y"],
                            z=elem["location"]["z"]
                        ),
                        rotation=Rotation(
                            pitch=elem["rotation"]["pitch"],
                            yaw=elem["rotation"]["yaw"],
                            roll=elem["rotation"]["roll"]
                        )
                    )
                )
    logger.debug(message=f"Found {len(lane_mid_points)} midpoints for lane {lane.id} of road {road.id}.")
    return lane_mid_points

def __build_speed_limits(lane: XODRRoadLaneSectionLCRLane, road: XODRRoad, landmarks: List[Landmark], logger: RcutilsLogger) -> List[SpeedLimit]:
    speed_limits: List[SpeedLimit] = []
    landmarks = [l for l in landmarks if l.type == LandmarkType.Maximum_Speed]
    for index, sign in enumerate(landmarks):
        if sign.s >= road.length:
            continue
        speed_limit_value: float = sign.value
        next_sign_location: float = road.length
        if index < len(speed_limits) - 1:
            next_speed_sign: SpeedLimit = speed_limits[index + 1]
            next_sign_location = next_speed_sign.from_distance_from_start
        speed_limits.append(SpeedLimit(speed_limit=speed_limit_value, from_distance_from_start=sign.s, to_distance_from_start=next_sign_location))

    return speed_limits

def __build_lane_list_from_lane_section(section: XODRRoadLaneSection, road: XODRRoad, intersecting_lanes: List[ContactLaneInfo],
                                        predecessor_lanes: List[ContactLaneInfo], successor_lanes: List[ContactLaneInfo],
                                        conctact_point_dict: Dict[int, Dict[int, Tuple[int, int, BaseGeometry]]], logger: RcutilsLogger) -> List[Lane]:
    def build_lane(lane: XODRRoadLaneSectionLCRLane) -> Lane:
        s: float = section.s
        lane_width: float = lane.widths[0].a if len(lane.widths) != 0 else float(0)
        midpoints_with_id: List[LaneMidpoint] = __build_lane_midpoints(lane=lane, road=road, logger=logger)
        midpoints: List[LaneMidpoint] = [midpoint for midpoint in midpoints_with_id if midpoint.road_id == road.id and midpoint.lane_id == lane.id]


        landmarks: List[Landmark] = __get_landmarks(road=road)

        return Lane(
            road_id=road.id,
            lane_id=lane.id,
            lane_type=__get_lane_type(lane_type=lane.type),
            s=s,
            lane_width=lane_width,
            lane_length=road.length,
            traffic_lights=__get_traffic_lights(road=road, current_lane=lane, logger=logger),
            landmarks=landmarks,
            predecessor_lanes=predecessor_lanes,
            successor_lanes=successor_lanes,
            intersecting_lanes=intersecting_lanes,
            lane_mid_points=midpoints,
            speed_limits=[], #TODO: It is not foreseen to export it? __build_speed_limits(lane=lane, road=road, landmarks=landmarks, logger=logger),
            contact_areas = __build_contact_areas(conctact_point_dict=conctact_point_dict, current_road=road.id, current_lane=lane.id, lane_midpoints=midpoints_with_id, logger=logger)
        )

    lane_list: List[Lane] = []
    if section.center is not None:
        for lane in section.center.lanes:
            if lane.id == 0: # ignore center lane
                continue
            lane_list.append(build_lane(lane))

    if section.left is not None:
        for lane in section.left.lanes:
            if lane.id == 0: # ignore center lane
                continue
            lane_list.append(build_lane(lane))

    if section.right is not None:
        for lane in section.right.lanes:
            if lane.id == 0: # ignore center lane
                continue
            lane_list.append(build_lane(lane))

    return lane_list

def __get_traffic_lights(road: XODRRoad, current_lane: XODRRoadLaneSectionLCRLane, logger: RcutilsLogger) -> List[StaticTrafficLight]: 
    traffic_lights: List[StaticTrafficLight] = []
    if len(road.signalReferences) != 0:
        for signal in road.signalReferences:
            if signal.validity.fromLane <= current_lane.id <= signal.validity.toLane:
                # we set stop location to signal location for now
                loc: Location = __build_signal_location(road=road, signal_ref=signal, logger=logger)
                traffic_lights.append(
                    StaticTrafficLight
                    (
                        # id=signal.id, removed for now
                        rotation=__build_rotation_of_rpy(roll=signal.roll, pitch=signal.pitch, yaw=0),
                        location=loc,
                        open_drive_id=str(signal.id),
                        position_distance=0.0,
                        stop_locations=[loc]
                    )
                )
    # test signals in addition to signal references
    logger.debug(message=f"{road.signals=}\t{road.signalReferences=}") if len(road.signals) != 0 or len(road.signalReferences) != 0 else None
    if len(road.signals) != 0:
        for signal in road.signals:
            if signal.name is None or not "light" in signal.name.lower():
                continue # simple sign here
            if signal.validity.fromLane <= current_lane.id <= signal.validity.toLane:
                # we set stop location to signal location for now
                loc: Location = __build_signal_location(road=road, signal_ref=signal, logger=logger)
                traffic_lights.append(
                    StaticTrafficLight
                    (
                        # id=signal.id, removed for now
                        rotation=__build_rotation_of_rpy(roll=signal.roll, pitch=signal.pitch, yaw=0),
                        location=loc,
                        open_drive_id=str(signal.id),
                        position_distance=0.0,
                        stop_locations=[loc]
                    )
                )
    return traffic_lights

def __build_signal_location(road: XODRRoad, signal_ref: XODRRoadSignalReference, logger: RcutilsLogger) -> Location:
    x,y = __get_absolute_position_from_relative_point(road=road, obj=signal_ref)
    return Location(x=x,y=y,z=0) # we ignore the lateral change for now

def __get_absolute_position_from_relative_point(road: XODRRoad, obj: Union[XODRRoadSignalReference, XODRRoadObjectsElement]):
    if road.planView.geometry is None or len(road.planView.geometry) == 0:
        raise XODRError(f"Invalid road geometry! No geometry was found for road with id {road.id}")
    # we can get the start of the road segment now.
    # it contains a absoulute point given by x and y.
    # We can use that to determine the x and y of a signal 
    # by using them and adding the s and t values in respectively
    # in relation to the heading hdg.
    x: Union[float, None] = road.planView.geometry[0].x
    y: Union[float, None] = road.planView.geometry[0].y
    hdg: Union[float, None] = road.planView.geometry[0].hdg
    if x is None or y is None or hdg is None:
        raise XODRError(f"Invalid road geometry! Absolute coordinates are missing for road {road.id}!")
    return (
            x + math.cos(hdg) * obj.s - math.sin(hdg) * obj.t,
            y + math.sin(hdg) * obj.s + math.cos(hdg) * obj.t
        )

def __build_rotation_of_rpy(roll: Union[float,None], pitch: Union[float,None], yaw: Union[float,None]) -> Rotation:
    pitch = pitch if pitch is not None else float(0)
    roll = roll if roll is not None else float(0)
    yaw = yaw if yaw is not None else float(0)
    return Rotation(pitch=pitch,yaw=yaw,roll=roll)

def __get_lane_type(lane_type: str) -> LaneType:
    for e in LaneType:
        if e.name == lane_type.capitalize():
            return e
    return LaneType.Driving


def __get_landmarks(road: XODRRoad) -> List[Landmark]:
    # just static signs
    #TODO add traffic light as dynamic landmarks the same way
    landmarks: List[Landmark] = []
    if not road.objects is None and not road.objects.elements is None and not len(road.objects.elements) == 0:
        for element in road.objects.elements:
            x, y = __get_absolute_position_from_relative_point(road=road, obj=element)
            landmarks.append(
                Landmark(
                    id=element.id,
                    road_id=road.id,
                    country="OpenDRIVE", # only signals have this set
                    distance=0.0,
                    h_offset=0.0, # only signals have this set
                    height=element.height if element.height is not None else 0.0,
                    is_dynamic=False, # static signs can never be dynamic
                    location=Location(x=x, y=y, z=0.0),
                    name=element.name,
                    orientation=__resolve_landmark_orientation_by_orientation(orientation=element.orientation),
                    pitch=element.pitch,
                    roll=element.roll,
                    rotation=__build_rotation_of_rpy(roll=element.roll, pitch=element.pitch, yaw=0.0),
                    s=element.s,
                    sub_type="",
                    text="", # static signs do not have xodr text???
                    type=__resolve_landmark_type_by_name(element.name),
                    unit="mph", # seems to be mph always
                    value=float(element.name[-2:]) if element.name[-2:].isdigit() else 0.0,
                    width=element.width,
                    z_offset=element.zOffset
                )
            )

    return landmarks

def __resolve_landmark_type_by_name(name: str) -> LandmarkType:
    if "Speed_" in name:
        return LandmarkType.Maximum_Speed
    return LandmarkType.Light_Post

def __resolve_landmark_orientation_by_orientation(orientation: str) -> LandmarkOrientation:
    if orientation == "+":
        return LandmarkOrientation.Positive
    elif orientation == "-":
        return LandmarkOrientation.Negative
    else:
        return LandmarkOrientation.Both

def __build_road_from_xodr(xodr_road: XODRRoad, is_junction: bool, intersecting_lanes: List[ContactLaneInfo],
    predecessor_lanes: List[ContactLaneInfo], successor_lanes: List[ContactLaneInfo], contact_point_dict: Dict[int, Dict[int, Tuple[int, int, BaseGeometry]]],
    logger: RcutilsLogger) -> Road:
    lane_list: List[Lane] = __build_lane_list_from_lane_section(section=xodr_road.lanes.laneSection, road=xodr_road, 
                                                                intersecting_lanes=intersecting_lanes, predecessor_lanes=predecessor_lanes,
                                                                successor_lanes=successor_lanes, conctact_point_dict=contact_point_dict, logger=logger)
    return Road(road_id=xodr_road.id, is_junction=is_junction, lanes=lane_list)

def __build_stars_road_blocks(xodr_roads: List[XODRRoad], removed_road_ids: Set[int], junctions: List[XODRJunction], logger: RcutilsLogger) -> List[Block]:
    road_blocks: List[Block] = []
    already_mapped: Dict[int, bool] = {}
    logger.debug(message=f"Junction Ids=[{','.join(str(junction.id) for junction in junctions)}]")

    predecessor_lanes: List[ContactLaneInfo] = []
    successor_lanes: List[ContactLaneInfo] = []

    for road in xodr_roads:
        predecessor_lanes, successor_lanes = __generate_dict_of_pred_succ_lanes(road=road)

    def find_intersecting_lanes(line_strings: Dict[int, Dict[int, LineString]]) -> Tuple[Dict[int, List[ContactLaneInfo]], Dict[int, Dict[int, Tuple[int, int, BaseGeometry]]]]:
        intersecting_lanes: Dict[int, List[ContactLaneInfo]] = {}
        # dict of roads containing entries for 
        #each lane and the possible contact points (Tuple=road,lane,point)
        intersect_dict: Dict[int, Dict[int, Tuple[int, int, BaseGeometry]]] = {}
        logger.debug(message=f"{str(line_strings)=}")
        for road, lanes in line_strings.items():
            for other_road, other_lanes in line_strings.items():
                if road == other_road:
                    continue
                for lane_id, line in lanes.items():
                    for other_lane_id, other_line in other_lanes.items():
                        logger.debug(message=f"Checking {str(line)} and {str(other_line)}")

                        if line.intersects(other=other_line):
                            if road not in intersecting_lanes:
                                intersecting_lanes[road] = [ContactLaneInfo(road_id=road, lane_id=lane_id)]
                            else:
                                intersecting_lanes[road].append(ContactLaneInfo(road_id=road, lane_id=lane_id))
                            intersect_dict[road] = {
                                lane_id: (other_road, other_lane_id, line.intersection(other_line))
                            }
            if road in intersecting_lanes:
                logger.debug(message=f"Found {len(intersecting_lanes[road])} intersecting lane(s) for road {road}.")
        return intersecting_lanes, intersect_dict
    
    intersecting_lanes, contact_point_dict = find_intersecting_lanes(line_strings=line_strings)

    logger.debug(message=", ".join(str(value) for key, value  in intersecting_lanes.items()))

    for junction in junctions:
        junction_roads: List[Road] = []
        block_name: str = "-".join(str(conn.connectingRoad) for conn in junction.connections if conn is not None \
                        and conn.connectingRoad not in removed_road_ids)
        for road in xodr_roads:
            if road.id in [int(value) for value in block_name.split("-")]:
                already_mapped[junction.id] = True
                junction_roads.append(__build_road_from_xodr(xodr_road=road, is_junction=True, intersecting_lanes=[],
                                                             predecessor_lanes=predecessor_lanes, successor_lanes=successor_lanes, 
                                                             contact_point_dict={}, logger=logger)
                                    )
        road_blocks.append(Block(id=block_name,roads=junction_roads))

    for road in xodr_roads:
        if road.id not in already_mapped.keys():
            dataclass_road: Road = __build_road_from_xodr(xodr_road=road, is_junction=False, intersecting_lanes=[],
                                                         predecessor_lanes=predecessor_lanes, successor_lanes=successor_lanes, 
                                                         contact_point_dict={}, logger=logger)
            road_blocks.append(
                Block(
                id=str(road.id),
                roads=[dataclass_road]
                )
            )

    def postprocess_missing_waypoints_for_empty_lanes(road_blocks: List[Block]) -> None:

        def calculate_shift_in_x_direction(lane: Lane) -> bool:
            if len(lane.lane_mid_points) >= 2:
                midpoint1: LaneMidpoint = lane.lane_mid_points[0]
                midpoint2: LaneMidpoint = lane.lane_mid_points[1]
                if midpoint1.location.x != midpoint2.location.x:
                    return True
            return False

        def shift_lane_midpoints(lane_with_midpoints: Lane, empty_lane: Lane) -> List[LaneMidpoint]:
            shifted_midpoints: List[LaneMidpoint] = []
            shift_x: bool = calculate_shift_in_x_direction(lane_with_midpoints)

            shift_width: float = (lane_with_midpoints.lane_width / 2) + (empty_lane.lane_width / 2)

            for midpoint in lane_with_midpoints.lane_mid_points:
                new_location: Location = replace(midpoint.location)
                if shift_x:
                    new_location.x = midpoint.location.x - shift_width if empty_lane.lane_id > 0 else midpoint.location.x + shift_width
                else:
                    new_location.y = midpoint.location.y - shift_width if empty_lane.lane_id > 0 else midpoint.location.y + shift_width
                new_midpoint: LaneMidpoint = replace(midpoint, location=new_location)
                shifted_midpoints.append(new_midpoint)
            logger.debug(f"Created {len(shifted_midpoints)} new LaneMidpoints for lane {empty_lane.lane_id} of road {empty_lane.road_id}.")
            return shifted_midpoints

        for block in road_blocks:
            if "-" in block.id: # we have all road waypoints in junctions
                continue
            for road in block.roads:

                positive_lanes = sorted([lane for lane in road.lanes if lane.lane_id > 0], key=lambda lane: lane.lane_id)
                # we want the abs(lane_id) to be increasing
                negative_lanes = list(reversed(sorted([lane for lane in road.lanes if lane.lane_id < 0], key=lambda lane: lane.lane_id)))#
                if len(positive_lanes) != 0:
                    for index, lane in enumerate(positive_lanes):
                        if index == 0: # should be the case for lane 1
                            continue
                    lane.lane_mid_points = shift_lane_midpoints(lane_with_midpoints=positive_lanes[index - 1], empty_lane=lane)
                if len(negative_lanes) != 0:
                    for index, lane in enumerate(iterable=negative_lanes):
                        if index == 0: # should be the case for lane 1
                            continue
                        lane.lane_mid_points = shift_lane_midpoints(lane_with_midpoints=negative_lanes[index - 1], empty_lane=lane)



    def postprocess_intersecting_lanes_in_blocks(road_blocks: List[Block], line_strings: Dict[int, Dict[int, LineString]]) -> None:
        logger.debug(f"{len(line_strings)} line strings found.")
        for key, value in line_strings.items():
            logger.debug(f"{str(key)}: {str(value)},\n")
        for block in road_blocks:
            if not "-" in block.id: # we have a straight road (no junction) here
                continue

            str_road_ids: str = ",".join(str(road.road_id) for road in block.roads)
            logger.debug(message=f"Block {block.id} contains roads {str_road_ids}.")

            for road, other_road in list(product(block.roads, block.roads)):
                if road.road_id == other_road.road_id:
                    continue
                if road.road_id not in line_strings.keys() or other_road.road_id not in line_strings.keys():
                        continue
                for lane, other_lane in list(product(road.lanes, other_road.lanes)):
                    if lane.lane_id == other_lane.lane_id:
                        continue
                    if lane.lane_id not in line_strings[road.road_id] or other_lane.lane_id not in line_strings[other_road.road_id]:
                        continue
                    if line_strings[road.road_id][lane.lane_id].intersects(other=line_strings[other_road.road_id][other_lane.lane_id]):
                        if lane.intersecting_lanes is None:
                            lane.intersecting_lanes = []
                        lane.intersecting_lanes.append(ContactLaneInfo(road_id=other_road.road_id, lane_id=other_lane.lane_id))

    # create lines of waypoints inside junctions for each road
    # later we can check if any intersect inside a junction 
    def create_linestrings_from_waypoints(road_blocks: List[Block]) -> Dict[int, Dict[int, LineString]]:

        line_strings_dict: Dict[int, Dict[int, LineString]] = {}

        for block in road_blocks:
            for road in block.roads:
                if not road.is_junction: # only needed for junctions
                    continue
                for lane in road.lanes:
                    line_string_tuple_list: List[Tuple[float, float]] = []
                    for midpoint in lane.lane_mid_points:
                        line_string_tuple_list.append((midpoint.location.x, midpoint.location.y))

                    if road.road_id not in line_strings_dict:
                        line_strings_dict[road.road_id] = {}

                    if lane.lane_id not in line_strings_dict[road.road_id]:
                        line_strings_dict[road.road_id][lane.lane_id] = LineString(line_string_tuple_list)

        logger.info(message=f"Created {len(line_strings_dict)} LineStrings from LaneMidpoints in junctions.")
        return line_strings_dict


    postprocess_missing_waypoints_for_empty_lanes(road_blocks=road_blocks)

    line_strings_from_method: Dict[int, Dict[int, LineString]] = create_linestrings_from_waypoints(road_blocks=road_blocks)

    postprocess_intersecting_lanes_in_blocks(road_blocks=road_blocks, line_strings=line_strings_from_method)

    # TODO CONTACT_POINT_DICT

    return road_blocks
