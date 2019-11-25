import numpy as np
from typing import List, Optional, Dict, Tuple

from camera.image_recognition import Point, PolarPoint

try:
    dataclass  # python 3.7.1
except:
    from dataclasses import dataclass


class StreamingMovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []
        self.sum = 0

    def __call__(self, value):
        if np.isnan(value):
            return value
        self.values.append(value)
        self.sum += value
        if len(self.values) > self.window_size:
            self.sum -= self.values.pop(0)
        return float(self.sum) / len(self.values)


Centimeter = float


@dataclass
class RecognitionState:
    """Class for keeping track of the recognition state."""
    balls: List[PolarPoint] = list
    goal_yellow: Optional[PolarPoint] = None
    goal_blue: Optional[PolarPoint] = None
    closest_edge: Optional[PolarPoint] = None
    angle_adjust: float = None
    h_bigger: int = None
    h_smaller: int = None
    field_contours: List[Tuple[int, int, int, int]] = None
    goal_yellow_rect: List[Tuple[int, int, int, int]] = None
    goal_blue_rect: List[Tuple[int, int, int, int]] = None

    @staticmethod  # for some reason type analysis didn't work for classmethod
    def from_dict(packet: dict) -> 'RecognitionState':
        balls: List[PolarPoint] = [PolarPoint(**b) for b in packet.get('balls', [])]
        goal_yellow: Optional[PolarPoint] = packet.get('goal_yellow') and PolarPoint(**packet['goal_yellow'])
        goal_blue: Optional[PolarPoint] = packet.get('goal_blue') and PolarPoint(**packet['goal_blue'])
        closest_edge: Optional[PolarPoint] = packet.get('closest_edge') and PolarPoint(**packet['closest_edge'])
        angle_adjust, h_bigger, h_smaller = packet.get('goal_angle_adjust')
        field_contours = packet.get('field_contours', [])
        goal_yellow_rect = packet.get('goal_yellow_rect', [])
        goal_blue_rect = packet.get('goal_blue_rect', [])

        return RecognitionState(
            balls, goal_yellow, goal_blue, closest_edge, angle_adjust, h_bigger, h_smaller,
            field_contours, goal_yellow_rect, goal_blue_rect)
