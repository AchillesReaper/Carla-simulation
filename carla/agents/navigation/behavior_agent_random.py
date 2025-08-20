# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


""" This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights,
traffic signs, and has different possible configurations. """

import random
import numpy as np
import carla
# from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.local_planner import RoadOption
from agents.navigation.behavior_types import Cautious, Aggressive, Normal, Random

from agents.tools.misc import get_speed, positive, is_within_distance, compute_distance

class RandomBehavior(object):
    """Class for Random agent."""
    max_speed               = random.randint(10, 150)
    speed_lim_dist          = random.randint(1, 10)
    speed_decrease          = random.randint(2, 15)
    safety_time             = random.randint(1, 5)
    min_proximity_threshold = random.randint(1, 15)
    braking_distance        = random.randint(1, 8)
    tailgate_counter        = 0

    # def __init__(self):
    #     self.max_speed               = random.randint(10, 150)
    #     self.speed_lim_dist          = random.randint(1, 10)
    #     self.speed_decrease          = random.randint(2, 15)
    #     self.safety_time             = random.randint(1, 5)
    #     self.min_proximity_threshold = random.randint(1, 15)
    #     self.braking_distance        = random.randint(1, 8)
    #     self.tailgate_counter        = 0

class RandomAgent(BehaviorAgent):
    """
    RandomAgent implements an agent based on behavior agent.
    """

    def __init__(self, vehicle, behavior='normal', opt_dict={}, map_inst=None, grp_inst=None):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param behavior: type of agent to apply
        """
        super().__init__(vehicle, opt_dict=opt_dict, map_inst=map_inst, grp_inst=grp_inst)
        self._behavior = Normal()


    def run_step(self, debug=False):
        """
        Execute one step of navigation.

            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """
        self._update_information()

        control = None
        if self._behavior.tailgate_counter > 0:
            self._behavior.tailgate_counter -= 1

        ego_vehicle_loc = self._vehicle.get_location()
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)

        # 1: Red lights and stops behavior
        # if self.traffic_light_manager():
        #     return self.emergency_stop()

        # 2.1: Pedestrian avoidance behaviors
        walker_state, walker, w_distance = self.pedestrian_avoid_manager(ego_vehicle_wp)

        if walker_state:
            # Distance is computed from the center of the two cars,
            # we use bounding boxes to calculate the actual distance
            distance = w_distance - max(
                walker.bounding_box.extent.y, walker.bounding_box.extent.x) - max(
                    self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)

            # Emergency brake if the car is very close.
            # if distance < self._behavior.braking_distance:
            if distance < RandomBehavior.braking_distance:
                return self.emergency_stop()

        # 2.2: Car following behaviors
        vehicle_state, vehicle, distance = self.collision_and_car_avoid_manager(ego_vehicle_wp)

        if vehicle_state:
            # Distance is computed from the center of the two cars,
            # we use bounding boxes to calculate the actual distance
            distance = distance - max(
                vehicle.bounding_box.extent.y, vehicle.bounding_box.extent.x) - max(
                    self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)

            # Emergency brake if the car is very close.
            # if distance < self._behavior.braking_distance:
            if distance < RandomBehavior.braking_distance:
                return self.emergency_stop()
            else:
                control = self.car_following_manager(vehicle, distance)

        # 3: Intersection behavior
        elif self._incoming_waypoint.is_junction and (self._incoming_direction in [RoadOption.LEFT, RoadOption.RIGHT]):
            target_speed = min([
                # self._behavior.max_speed,
                RandomBehavior.max_speed,
                self._speed_limit - 5])
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        # 4: Normal behavior
        else:
            target_speed = min([
                # self._behavior.max_speed,
                # self._speed_limit - self._behavior.speed_lim_dist
                RandomBehavior.max_speed,
                self._speed_limit - RandomBehavior.speed_lim_dist
            ])
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        return control

    def emergency_stop(self):
        """
        Overwrites the throttle a brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns

            :param speed (carl.VehicleControl): control to be modified
        """
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control
