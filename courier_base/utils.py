import math
from typing import List, Tuple, Optional

import numpy as np


def circles_generator(x0: float, y0: float, velocity: float, max_time: float, delay: float = 0):
    """
        Generator for circles given center, radidus as a function of (velocity, time)
        :param x0: - x coordinate of center, meters
        :param y0: - y coordinate of center, meters
        :param velocity: speed of agent, meters / minute
        :param max_time: minutes, maximal time of trajectories (number of yielding traj = max_time)
        :param delay: delay after that agent will accomplish current work, minutes
        :yield : circle in canonical form
    """
    for time in np.arange(0, max_time, 1):
        yield x0, y0, max(velocity * (time-delay), 0)


def get_intercetions(c0: Tuple[float, float, float], c1: Tuple[float, float, float])\
                     -> Optional[List]:
    """
    :param c0: x, y, radius of first circle
    :param c1: x, y, radius of second circle
    :return: Optional[List[List[x, y], List[x, y]]] - tow points of circles intersections, or None of no intersection
    """

    x0, y0, r0 = c0
    x1, y1, r1 = c1
    d = math.sqrt((x1-x0)**2 + (y1-y0)**2)

    # non intersecting
    if d > r0 + r1:
        return None

    # One circle within other
    if d < abs(r0-r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)

        h = math.sqrt(r0 ** 2 - a ** 2+0.01)

        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return [[x3, y3], [x4, y4]]


def distance_btwn_points(p0: List[float], p1: List[float]) -> float:
    """

    :param p0: coords of first point: x, y
    :param p1: coords of second point: x, y
    :return: l2 norm btwn points
    """

    p0 = np.array(p0)
    p1 = np.array(p1)
    return np.linalg.norm(p0-p1)


def get_best_mp(robot_pose: np.ndarray, worker_pose: np.ndarray, delay_robot, delay_person, robot_speed,
                worker_speed, max_time, goal) -> np.ndarray:
    """

    :param robot_pose: position of robot: x, y
    :param worker_pose: position of worker: x, y
    :param delay_robot: delay after that agent will accomplish current work, minutes
    :param delay_person: delay after that agent will accomplish current work, minutes
    :param robot_speed: mean speed of robot
    :param worker_speed: mean speed of worker
    :param max_time: minutes, maximal time of trajectories (number of yielding traj = max_time)
    :param goal: position of goal (order address)
    :return:  np.ndarray, point of intersection
    """

    if np.linalg.norm(robot_pose-worker_pose) < 0.1:
        return robot_pose

    intersections = [get_intercetions(x, y) for x, y in zip(circles_generator(robot_pose[0], robot_pose[1],
                                                                              robot_speed, max_time, delay_robot),
                                                            circles_generator(worker_pose[0], worker_pose[1],
                                                                              worker_speed, max_time, delay_person))]

    intersections = list(filter(None, intersections))

    if  (len(intersections) == 0):
        # print("No intersections Found! {robot_pose}, {worker_pose}, {delay_robot},{delay_person}, {robot_speed},"
        #        " {worker_speed}, {max_time}, {goal}".format(
        #                             robot_pose=robot_pose, worker_pose=worker_pose, delay_robot=delay_robot,
        #                             delay_person=delay_person, robot_speed=robot_speed, worker_speed=worker_speed,
        #                             max_time=max_time, goal=goal))
        return None
    intersections = [item for sublist in intersections for item in sublist]
    distances = []
    for (x, y) in intersections:
        distances.append(distance_btwn_points([x, y], goal))


    mp = intersections[np.argmin(distances)]
    initial_distance = np.linalg.norm(worker_pose - goal)
    mp_distance = np.linalg.norm(mp - goal)


    # if (initial_distance < mp_distance):
    #     print("initial_distance {initial_distance}, mp_distance {mp_distance}".format(mp_distance=mp_distance, initial_distance=initial_distance))

    return np.array(mp)

