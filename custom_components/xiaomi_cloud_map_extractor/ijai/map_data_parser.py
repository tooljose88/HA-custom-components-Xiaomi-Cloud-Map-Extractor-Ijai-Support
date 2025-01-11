"""Ijai map parser."""

import logging
import math
import os
import sys
import zlib
from typing import Any

import custom_components.xiaomi_cloud_map_extractor.ijai.RobotMap_pb2 as RobotMap
from custom_components.xiaomi_cloud_map_extractor.common.map_data import *
from custom_components.xiaomi_cloud_map_extractor.common.map_data import (
    Area, ImageData, MapData, Path, Point, Room, Wall, Zone)
from custom_components.xiaomi_cloud_map_extractor.common.map_data_parser import \
    MapDataParser
from custom_components.xiaomi_cloud_map_extractor.types import (Colors,
                                                                Drawables,
                                                                ImageConfig,
                                                                Sizes, Texts)

from custom_components.xiaomi_cloud_map_extractor.ijai.aes_decryptor import decrypt
from custom_components.xiaomi_cloud_map_extractor.ijai.image_handler import ImageHandlerIjai

_LOGGER = logging.getLogger(__name__)


class MapDataParserIjai(MapDataParser):
    POSITION_UNKNOWN = 1100
    robot_map = RobotMap.RobotMap()

    def unpack_map(raw_encoded: bytes, **kwargs) -> bytes:
        return zlib.decompress(
            decrypt(
                raw_encoded, 
                kwargs['wifi_sn'], 
                kwargs['owner_id'], 
                kwargs['device_id'], 
                kwargs['model'], 
                kwargs['device_mac']))

    @staticmethod
    def parse(raw: bytes, colors: Colors, drawables: Drawables, texts: Texts, sizes: Sizes,
              image_config: ImageConfig, *args, **kwargs) -> MapData:
        map_data = MapData(0, 4)
        MapDataParserIjai.robot_map = RobotMap.RobotMap()
        MapDataParserIjai.robot_map.ParseFromString(raw)

        if hasattr(MapDataParserIjai.robot_map, "mapData"):
            map_data.image, map_data.rooms, map_data.cleaned_rooms = MapDataParserIjai.parse_image(MapDataParserIjai.robot_map, colors, image_config, DRAWABLE_CLEANED_AREA in drawables)

        if hasattr(MapDataParserIjai.robot_map, "historyPose"):
            map_data.path = MapDataParserIjai.parse_history()

        if hasattr(MapDataParserIjai.robot_map, "chargeStation"):
            pos_info = MapDataParserIjai.robot_map.chargeStation
            map_data.charger = Point(x = pos_info.x, y = pos_info.y, a = pos_info.phi * 180 / math.pi)
            _LOGGER.debug("pos: %s", map_data.charger)

        if hasattr(MapDataParserIjai.robot_map, "currentPose"):
            pos_info = MapDataParserIjai.robot_map.currentPose
            map_data.vacuum_position = Point(x = pos_info.x, y = pos_info.y, a = pos_info.phi * 180 / math.pi)
            _LOGGER.debug("pos: %s", map_data.vacuum_position)

        if hasattr(MapDataParserIjai.robot_map, "mapInfo") and hasattr(MapDataParserIjai.robot_map, "roomDataInfo") and map_data.rooms is not None:
            MapDataParserIjai.parse_rooms(MapDataParserIjai.robot_map, map_data.rooms)

        if hasattr(MapDataParserIjai.robot_map, "virtualWalls") :
            map_data.walls, map_data.no_go_areas, map_data.no_mopping_areas = MapDataParserIjai.parse_restricted_areas(MapDataParserIjai.robot_map)


        if map_data.rooms is not None:
            _LOGGER.debug('rooms: %s', [str(room) for number, room in map_data.rooms.items()])
        if not map_data.image.is_empty:
            MapDataParserIjai.draw_elements(colors, drawables, sizes, map_data, image_config)
            if len(map_data.rooms) > 0 and map_data.vacuum_position is not None:
                vacuum_position_on_image = MapDataParserIjai.map_to_image(map_data.vacuum_position)
                map_data.vacuum_room = MapDataParserIjai.get_current_vacuum_room(MapDataParserIjai.robot_map.mapData.mapData, vacuum_position_on_image, MapDataParserIjai.robot_map.mapHead.sizeX)
                if map_data.vacuum_room is not None:
                    map_data.vacuum_room_name = map_data.rooms[map_data.vacuum_room].name
                _LOGGER.debug('current vacuum room: %s', map_data.vacuum_room)
            ImageHandlerIjai.rotate(map_data.image)
            ImageHandlerIjai.draw_texts(map_data.image, texts)
        return map_data

    @staticmethod
    def parse_image(robot_map: RobotMap, colors: Colors, image_config: ImageConfig, draw_cleaned_area: bool) -> tuple[ImageData, dict[int, Room], set[int]]:
        image_left = 0
        image_top = 0
        image_width = robot_map.mapHead.sizeX
        image_height = robot_map.mapHead.sizeY
        image_size = image_height * image_width
        _LOGGER.debug("width: %d, height: %d", image_width, image_height)

        if image_width \
                - image_width * (image_config[CONF_TRIM][CONF_LEFT] + image_config[CONF_TRIM][CONF_RIGHT]) / 100 \
                < MINIMAL_IMAGE_WIDTH:
            image_config[CONF_TRIM][CONF_LEFT] = 0
            image_config[CONF_TRIM][CONF_RIGHT] = 0
        if image_height \
                - image_height * (image_config[CONF_TRIM][CONF_TOP] + image_config[CONF_TRIM][CONF_BOTTOM]) / 100 \
                < MINIMAL_IMAGE_HEIGHT:
            image_config[CONF_TRIM][CONF_TOP] = 0
            image_config[CONF_TRIM][CONF_BOTTOM] = 0

        image, rooms_raw, cleaned_areas, cleaned_areas_layer = ImageHandlerIjai.parse(robot_map.mapData.mapData, image_width, image_height,
                                                                                      colors, image_config, draw_cleaned_area)
        if image is None:
            image = MapDataParserIjai._image_generator.create_empty_map_image()
        _LOGGER.debug("img: number of rooms: %d, numbers: %s", len(rooms_raw), rooms_raw.keys())
        rooms = {}
        for number, room in rooms_raw.items():
            rooms[number] = Room(number,
                MapDataParserIjai.image_to_map(room[0] + image_left),
                MapDataParserIjai.image_to_map(room[1] + image_top),
                MapDataParserIjai.image_to_map(room[2] + image_left),
                MapDataParserIjai.image_to_map(room[3] + image_top),
            )
        return (
            ImageData(
                image_size,
                image_top,
                image_left,
                image_height,
                image_width,
                image_config,
                image,
                MapDataParserIjai.map_to_image,
                additional_layers={DRAWABLE_CLEANED_AREA: cleaned_areas_layer},
            ),
            rooms,
            cleaned_areas,
        )

    @staticmethod
    def parse_history() -> Path:
        path_points = []
        for pt in MapDataParserIjai.robot_map.historyPose.points:
            path_points.append(Point(x = pt.x, y = pt.y))
        return Path(len(path_points), 1, 0, [path_points])

    @staticmethod
    def parse_restricted_areas(robot_map: RobotMap) -> tuple[list[Wall], list[Area]]:
        virtual_walls = []
        no_go_zones = []
        no_mopping_zones = []

        for virtualWall in robot_map.virtualWalls:
            points = virtualWall.points 
            if len(points) >= 4:
                p1 = points[0]
                p2 = points[1]
                p3 = points[2]
                p4 = points[3]

                _LOGGER.debug("restricted: %s %s %s %s", p1, p2, p3, p4)
                if virtualWall.type == 2:
                    virtual_walls.append(Wall(p1.x, p1.y, p3.x, p3.y))
                elif virtualWall.type == 3:
                    no_go_zones.append(Area(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y))
                elif virtualWall.type == 6:
                    no_mopping_zones.append(Area(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y))

        return virtual_walls, no_go_zones, no_mopping_zones

    @staticmethod
    def parse_rooms(robot_map: RobotMap, map_data_rooms: dict[int, Room]) -> None:
        map_id = robot_map.mapHead.mapHeadId
        for map_data in robot_map.mapInfo:
            if (map_data.mapHeadId == map_id):
                current_map = map_data
                break
        map_name = current_map.mapName
        _LOGGER.debug("map#%d: %s", current_map.mapHeadId, map_name)
        for r in robot_map.roomDataInfo:
            if map_data_rooms is not None and r.roomId in map_data_rooms:
                map_data_rooms[r.roomId].name = r.roomName
                map_data_rooms[r.roomId].pos_x = r.roomNamePost.x
                map_data_rooms[r.roomId].pos_y = r.roomNamePost.y

            room_text_pos = Point(r.roomNamePost.x, r.roomNamePost.y)
            _LOGGER.debug("room#%d: %s %s", r.roomId, r.roomName, room_text_pos)

    @staticmethod
    def get_current_vacuum_room(map_data: bytes, vacuum_position_on_image: Point, image_width: int) -> int | None:
        _LOGGER.debug(f"pos on image: {vacuum_position_on_image}")
        pixel_type = map_data[int(vacuum_position_on_image.y) * image_width + int(vacuum_position_on_image.x)]
        if ImageHandlerIjai.MAP_ROOM_MIN <= pixel_type <= ImageHandlerIjai.MAP_ROOM_MAX:
            return pixel_type
        if ImageHandlerIjai.MAP_SELECTED_ROOM_MIN <= pixel_type <= ImageHandlerIjai.MAP_SELECTED_ROOM_MAX:
            return pixel_type - ImageHandlerIjai.MAP_SELECTED_ROOM_MIN + ImageHandlerIjai.MAP_ROOM_MIN
        return None

    @staticmethod
    def map_to_image(p: Point) -> Point:
        return Point(p.x * 20 + 400, p.y * 20 + 400)

    @staticmethod
    def image_to_map(x: float) -> float:
        return (x - 400) / 20