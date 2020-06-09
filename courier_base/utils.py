import math
from typing import List, Tuple

import numpy as np


def circles_generator(x0: float, y0: float, velocity: float, max_time: float, delay: float=0):

    for time in np.arange(0, max_time):
        yield x0, y0, max(velocity * (time-delay), 0)


def get_intercetions(c0: Tuple[float, float, float], c1: Tuple[float, float, float]):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

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
        a = (r0 ** 2- r1 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(r0 ** 2 - a **2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return [[x3, y3], [x4, y4]]


def distance_btwn_points(p0: List[float], p1: List[float]) -> float:

    p0 = np.array(p0)
    p1 = np.array(p1)
    return np.linalg.norm(p0-p1)


def get_best_mp(robot_pose: np.ndarray, worker_pose: np.ndarray, delay_robot, delay_person, robot_speed, worker_speed, max_time, goal):

    if np.linalg.norm(robot_pose-worker_pose) < 0.1:
        return robot_pose

    intersections = [get_intercetions(x, y) for x, y in zip(circles_generator(robot_pose[0], robot_pose[1],
                                                                              robot_speed, max_time, delay_robot),
                                                            circles_generator(worker_pose[0], worker_pose[1],
                                                                              worker_speed, max_time, delay_person))]

    intersections = list(filter(None, intersections))

    assert len(intersections) > 0, "No intersections Found! {robot_pose}, {worker_pose}, {delay_robot}," \
                                   " {delay_person}, {robot_speed}, {worker_speed}, {max_time}, {goal}".format(
                                    robot_pose=robot_pose, worker_pose=worker_pose, delay_robot=delay_robot,
                                    delay_person=delay_person, robot_speed=robot_speed, worker_speed=worker_speed,
                                    max_time=max_time, goal=goal)
    intersections = [item for sublist in intersections for item in sublist]
    distances = []
    for (x, y) in intersections:
        distances.append(distance_btwn_points([x, y], goal))


    mp = intersections[np.argmin(distances)]

    return mp

#
# if __name__ == "__main__":
#
#     robot_pose = [100, 100]
#     worker_pose = [0, 0]
#     robot_speed = 5
#     worker_speed = 6
#     max_time = 16
#     goal = [75, 50]
#
#     mp = get_best_mp(robot_pose, worker_pose, 0, 0, robot_speed, worker_speed, max_time, goal)
#     print(mp)
