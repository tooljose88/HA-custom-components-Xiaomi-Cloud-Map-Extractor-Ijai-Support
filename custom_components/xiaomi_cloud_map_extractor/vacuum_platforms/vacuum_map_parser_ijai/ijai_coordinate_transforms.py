import sys
import os

from vacuum_map_parser_base.map_data import Point
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import RobotMap_pb2 as RobotMap

class Transformer:
    def __init__(self, map:RobotMap):
        self.map_head = map.mapHead
        self.to_image_multiplier = Point(self.map_head.sizeX/(self.map_head.maxX - self.map_head.minX), 
                                        self.map_head.sizeY/(self.map_head.maxY - self.map_head.minY))
    def map_to_image(self, pt:Point):
        return Point((pt.x -self.map_head.minX) * self.to_image_multiplier.x, 
                     (pt.y - self.map_head.minY) * self.to_image_multiplier.y)

    def image_to_map_x (self, x:int):
        return (x/self.to_image_multiplier.x + self.map_head.minX)
    
    def image_to_map_y (self, y:int):
        return (y/self.to_image_multiplier.y + self.map_head.minY)
