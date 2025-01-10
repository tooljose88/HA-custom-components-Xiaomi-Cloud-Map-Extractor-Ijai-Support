"""Ijai map image parser."""

import logging

from PIL import Image
from PIL.Image import Image as ImageType
from PIL.Image import Resampling

from vacuum_map_parser_base.config.color import ColorsPalette, SupportedColor
from vacuum_map_parser_base.config.drawable import Drawable
from vacuum_map_parser_base.config.image_config import ImageConfig
from vacuum_map_parser_base.map_data import Point

_LOGGER = logging.getLogger(__name__)


class IjaiImageParser:
    
    """Ijai map image parser."""

    MAP_OUTSIDE = 0x00
    MAP_WALL = 0xFF
    MAP_SCAN = 0x01
    MAP_NEW_DISCOVERED_AREA = 0x02
    MAP_ROOM_MIN = 10
    MAP_ROOM_MAX = 59
    MAP_SELECTED_ROOM_MIN = 60
    MAP_SELECTED_ROOM_MAX = 109

    def __init__(self, palette: ColorsPalette, image_config: ImageConfig, drawables: list[Drawable]):
        self._palette = palette
        self._image_config = image_config
        self._drawables = drawables
        self.color_map = {
                    IjaiImageParser.MAP_OUTSIDE: palette.get_color(SupportedColor.MAP_OUTSIDE),
                    IjaiImageParser.MAP_WALL:palette.get_color(SupportedColor.MAP_WALL_V2),
                    IjaiImageParser.MAP_SCAN: palette.get_color(SupportedColor.SCAN),
                    IjaiImageParser.MAP_NEW_DISCOVERED_AREA: palette.get_color(SupportedColor.NEW_DISCOVERED_AREA)}
    def parse(
        self, map_data: bytes, width: int, height: int
    ) -> tuple[ImageType | None, dict[int, tuple[int, int, int, int]], set[int], ImageType | None]:
        rooms = {}
        cleaned_areas = set()
        _LOGGER.debug(f"ijai parser: image_config = {self._image_config}")
        scale = self._image_config.scale
        trim_left = int(self._image_config.trim.left * width / 100)
        trim_right = int(self._image_config.trim.right * width / 100)
        trim_top = int(self._image_config.trim.top * height / 100)
        trim_bottom = int(self._image_config.trim.bottom * height / 100)
        trimmed_height = height - trim_top - trim_bottom
        trimmed_width = width - trim_left - trim_right
        if trimmed_width == 0 or trimmed_height == 0:
            return None, {}, set(), None

        image = Image.new('RGBA', (trimmed_width, trimmed_height))
        pixels = image.load()
        cleaned_areas_layer = None
        cleaned_areas_pixels = None
        draw_cleaned_area = Drawable.CLEANED_AREA in self._drawables
        if draw_cleaned_area:
            cleaned_areas_layer = Image.new("RGBA", (trimmed_width, trimmed_height))
            cleaned_areas_pixels = cleaned_areas_layer.load()
        _LOGGER.debug(f"trim_bottom = {trim_bottom}, trim_top = {trim_top}, trim_left = {trim_left}, trim_right = {trim_right}")
        unknown_pixels = set()
        for img_y in range(trimmed_height):
            y = trimmed_height - 1 - img_y
            for img_x in range(trimmed_width):
                x = img_x
                pixel_type = map_data[(img_y + trim_bottom)*width + x + trim_left]
                if pixel_type in self.color_map.keys():
                    pixels[x, y] = self.color_map[pixel_type]
                elif IjaiImageParser.MAP_ROOM_MIN <= pixel_type <= IjaiImageParser.MAP_SELECTED_ROOM_MAX:
                    room_x = img_x + trim_left
                    room_y = img_y + trim_bottom
                    room_number = pixel_type
                    if pixel_type >= IjaiImageParser.MAP_SELECTED_ROOM_MIN:
                        room_number = pixel_type - IjaiImageParser.MAP_SELECTED_ROOM_MIN + IjaiImageParser.MAP_ROOM_MIN
                        cleaned_areas.add(room_number)
                        if draw_cleaned_area:
                            cleaned_areas_pixels[x, y] = IjaiImageParser.get_color(SupportedColor.CLEANED_AREA)
                    rooms[room_number] = (room_x, room_y, room_x, room_y) \
                    if room_number not in rooms \
                    else (min(rooms[room_number][0], room_x),
                        min(rooms[room_number][1], room_y),
                        max(rooms[room_number][2], room_x),
                        max(rooms[room_number][3], room_y))
                    pixels[x, y] = self._palette.get_room_color(room_number)
                else:
                    pixels[x, y] = IjaiImageParser.get_color(SupportedColor.UNKNOWN)
                    unknown_pixels.add(pixel_type)
                    _LOGGER.debug(f"unknown pixel [{x},{y}] = {pixel_type}")
        if self._image_config.scale != 1 and trimmed_width != 0 and trimmed_height != 0:
            image = image.resize((int(trimmed_width * scale), int(trimmed_height * scale)), resample=Resampling.NEAREST)
            if draw_cleaned_area:
                cleaned_areas_layer = cleaned_areas_layer.resize(
                    (int(trimmed_width * scale), int(trimmed_height * scale)), resample=Image.NEAREST)
        if len(unknown_pixels) > 0:
            _LOGGER.warning('unknown pixel_types: %s', unknown_pixels)
        return image, rooms, cleaned_areas, cleaned_areas_layer

    @staticmethod
    def get_current_vacuum_room(map_data: bytes, vacuum_position_on_image: Point, image_width: int) -> int | None:
        _LOGGER.debug(f"pos on image: {vacuum_position_on_image}")
        pixel_type = map_data[int(vacuum_position_on_image.y) * image_width + int(vacuum_position_on_image.x)]
        if IjaiImageParser.MAP_ROOM_MIN <= pixel_type <= IjaiImageParser.MAP_ROOM_MAX:
            return pixel_type
        if IjaiImageParser.MAP_SELECTED_ROOM_MIN <= pixel_type <= IjaiImageParser.MAP_SELECTED_ROOM_MAX:
            return pixel_type - IjaiImageParser.MAP_SELECTED_ROOM_MIN + IjaiImageParser.MAP_ROOM_MIN
        return None
