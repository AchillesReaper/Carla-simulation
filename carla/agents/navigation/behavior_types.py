# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" 
This module contains the different parameters sets for each behavior. 
- max_speed: The maximum speed the agent is allowed to drive (in km/h or m/s, depending on implementation).
- speed_lim_dist: The distance from a speed limit sign at which the agent starts to adjust its speed.
- speed_decrease: The amount by which the agent reduces its speed when approaching a speed limit or obstacle.
- safety_time: The minimum time gap (in seconds) the agent maintains from the vehicle ahead for safety.
- min_proximity_threshold: The minimum distance (in meters) the agent keeps from other vehicles to avoid collisions.
- braking_distance: The distance (in meters) required for the agent to safely come to a stop.
- tailgate_counter: A counter used to track or manage tailgating behavior (e.g., how often the agent is too close to the vehicle ahead).
"""
import random

class Cautious(object):
    """Class for Cautious agent."""
    max_speed = 40
    speed_lim_dist = 6
    speed_decrease = 12
    safety_time = 3
    min_proximity_threshold = 12
    braking_distance = 6
    tailgate_counter = 0


class Normal(object):
    """Class for Normal agent."""
    max_speed = 50
    speed_lim_dist = 3
    speed_decrease = 10
    safety_time = 3
    min_proximity_threshold = 10
    braking_distance = 5
    tailgate_counter = 0


class Aggressive(object):
    """Class for Aggressive agent."""
    max_speed = 70
    speed_lim_dist = 1
    speed_decrease = 8
    safety_time = 3
    min_proximity_threshold = 8
    braking_distance = 4
    tailgate_counter = -1

class Random(object):
    """Class for Random agent."""
    max_speed                = random.randint(30, 150)
    speed_lim_dist           = 2
    speed_decrease           = 10
    safety_time              = 3
    min_proximity_threshold  = random.randint(5, 15)
    braking_distance         = 5
    tailgate_counter         = 0