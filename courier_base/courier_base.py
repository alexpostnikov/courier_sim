from random import seed, randint
from typing import List

import numpy as np
import simpy

from .utils import get_best_mp

LOGGING_LEVEL = 0
seed(1)
np.random.seed(1)

# TODO: find closest person (+ check when they will freed)
# TODO: find optimal meeting point? (depend on both positions, fraction(workers/robots), order address)

class OrderGenerator:
    """
    generation of orders to be performed
    :param env:  - simpy.Environment
    :param params: dict of params from yaml configuration file
    """

    def __init__(self, env: simpy.Environment, params: dict):
        self.env = env
        self.store = simpy.Store(env, capacity=1000000)

        self.order_counter = 0
        self.params = params
        for i in range(100):
            order = Order(self.env, self.order_counter, self.params)
            self.order_counter += 1
            self.store.put(order)
        self.store_gen_proc = self.env.process(self.gen_proc())



    def gen_proc(self):
        """
        process - generator yielding new orders given random (bounded) frequency
        """
        while True:
            t = max(np.random.normal(self.params["orders"]["mean"], self.params["orders"]["var"]), 0.1)
            yield self.env.timeout(t)
            self.order_counter += 1
            order = Order(self.env, self.order_counter, self.params)
            yield self.store.put(order)

            if LOGGING_LEVEL > 1:
                print("\t Order_generator: orders in queue:", len(self.store.items))


class Order:
    """
    :param env: the simpy environment
    :param id : unique id number of order
    :param params: dict of params from yaml configuration file
    """

    def __init__(self, env: simpy.Environment, id: int, params: dict):
        self.env = env

        self.address = np.random.uniform(low=params["area"][0], high=params["area"][1], size=2)
        self.executor = None
        self.started = self.env.event()
        self.ended = self.env.event()
        self.order_ctrl_proc = env.process(self.order_ctrl())
        self.id = id

    def order_ctrl(self):
        """
        logging the status of order
        """

        while True:
            if LOGGING_LEVEL > 2:
                print('\t\t Order {id}: order. created at {time:.2f}'.format(id=self.id, time=self.env.now))
            yield self.started
            if LOGGING_LEVEL > 2:
                print("\t\t Order {id}: order started at {time:.2f}".format(id=self.id, time=self.env.now))

            yield self.ended
            if LOGGING_LEVEL > 2:
                print("\t\t Order {id}: done at {time:.2f}".format(id=self.id, time=self.env.now))
            break


class Worker:
    """
    The person (worker) that bring orders to customer
    :param env - simpy.Environment
    :param id : unique id number of worker
    :param params: dict of params from yaml configuration file
    """

    def __init__(self, env: simpy.Environment, order_generator, id: int, params: dict):
        self.env = env
        self.pose = np.array([0., 0.])
        self.goal = np.array([0., 0.])
        self.mean_velocity = np.array(params["workers_speed"])*60  # meters/minute

        self.order_generator = order_generator
        if params["policy"] == "solo":
            self.work_process = env.process(self.take_order())
        else:
            # self.obey_robot_proc = self.env.event()
            # self.obey_proc = self.env.process(self.obey_robot())
            self.obey_proc = self.env.process(self.work())
        self.new_work_reactivate = env.event()
        self.orders_done = 0
        self.resourse = simpy.Resource(self.env, capacity=1)

        self.request = None
        self.robot = None
        self.order = None
        self.id = id
        self.idle_time = []
        self.walking_to_take_order_time = []
        self.walking_to_order_address_time = []
        self.idle_start = 0
        self.order_list = []
        self.busy_until = self.env.now
        self.end_pose = self.pose

    # if working without robots
    def take_order(self):
        """
         start new order processing (solo policy) if previous task is done
        """
        while 1:
            # Wait for order to be created
            self.idle_start = self.env.now
            order = yield self.order_generator.store.get()
            self.idle_time.append(self.env.now - self.idle_start)

            # Go to order address
            self.goal = order.address
            time_to_goal = np.linalg.norm(self.goal-self.pose) / self.mean_velocity
            self.walking_to_take_order_time.append(time_to_goal)
            yield self.env.timeout(time_to_goal)

            self.pose = self.goal
            order.ended.succeed()
            self.walking_to_order_address_time.append(time_to_goal)

            # Go back to base
            yield self.env.timeout(time_to_goal)
            self.pose = np.array([0., 0.])
            if LOGGING_LEVEL > 2:
                print("\t\tWorker{id}: at home home{time:.2f}")
            self.orders_done += 1

    def obey_robot(self):
        """
        start new order processing (robot policy) if previous task is done
        """
        while 1:
            self.idle_start = self.env.now
            if self.robot is not None:
                self.idle_time.append(self.env.now - self.idle_start)
                self.idle_start = None
                if LOGGING_LEVEL > 1:
                    print("\t\tWorker{id}: obeying! order id{order_id} {time:.2f}".format(id=self.id,
                                                                                      order_id=self.order.id,
                                                                                      time=self.env.now))
                self.request = self.resourse.request()

                self.walking_to_take_order_time.append(self.go_to_point(self.goal))
                yield self.go_to_point(self.goal)

                self.pose = self.goal
                self.walking_to_order_address_time.append(self.go_to_point(self.order.address))
                yield self.go_to_point(self.order.address)
                self.pose = self.order.address
                self.order = None
                self.resourse.release(self.request)
                if LOGGING_LEVEL > 1:
                    print("\t\tWorker{id}: it was pleasure{time:.2f}!".format(id=self.id, time=self.env.now))
                self.robot = None
                self.orders_done += 1

            else:
                yield self.env.timeout(0.5)
                if self.idle_start is not None:
                    self.idle_time.append(self.env.now - self.idle_start)
                    self.idle_start = None

    def go_to_point(self, meet_point: np.array):
        """
        generator yielding timeout for moving from one point to other
        :param meet_point:
        """

        self.goal = meet_point
        time_to_goal = np.linalg.norm(self.goal - self.pose) / self.mean_velocity
        return self.env.timeout(time_to_goal)

    def try_new_order(self, order, robot):

        delay = max(self.busy_until - self.env.now, 0)
        mp = get_best_mp(robot_pose=robot.pose, worker_pose=self.pose, delay_robot=0, delay_person=delay,
                         robot_speed=robot.mean_velocity, worker_speed=self.mean_velocity, max_time=100,
                         goal=order.address)
        if mp is None:
            return np.inf, None
        time_to_mp = np.linalg.norm(self.end_pose - mp) / self.mean_velocity
        time_to_order = np.linalg.norm(mp - order.address) / self.mean_velocity
        return max(self.busy_until, self.env.now) + time_to_mp + time_to_order, mp

    def add_new_order(self, robot, order, mp, new_busy_until):
        self.order_list.append({"order": order, "mp": mp, "robot": robot})
        self.busy_until = new_busy_until
        self.end_pose = order.address

    def work(self):
        yield self.env.timeout(0.1)
        while 1:
            if len(self.order_list) == 0:
                # print("\t worker{id} not doing anything, order len {len}, en".format(id=self.id,
                #len=len(self.order_list)))
                timeout = 1
                # if np.linalg.norm(self.pose - np.array([0, 0]) > 400):
                #     self.pose = self.pose + (-self.pose) /np.linalg.norm(self.pose) * self.mean_velocity * timeout
                yield self.env.timeout(timeout)
                self.idle_time.append(timeout)
            else:
                task = self.order_list.pop(0)
                time_to_mp = np.linalg.norm(self.pose - task["mp"]) / self.mean_velocity
                self.walking_to_take_order_time.append(time_to_mp)
                yield self.env.timeout(time_to_mp)
                self.pose = task["mp"]

                time_to_order = np.linalg.norm(self.pose - task["order"].address) / self.mean_velocity
                yield self.env.timeout(time_to_order)
                self.pose = task["order"].address
                self.orders_done += 1




class Monitor:
    """
    monitoring current state
    now support: mean  productivity calculating
    """

    def __init__(self, env: simpy.Environment, workers: List[Worker], params: dict):
        self.env = env
        self.workers = workers
        self.order_ctrl_proc = env.process(self.order_ctrl())

    def order_ctrl(self):
        while 1:
            yield self.env.timeout(60)
            total = self.total_orders_done
            print("\t Monitor: mean productivity (orders per hour) {prod:.2f}: ".format(prod=total / self.env.now*60))

    @property
    def total_orders_done(self):
        total = sum([worker.orders_done for worker in self.workers])
        return total

class Robot:

    """
        The person (worker) that bring orders to customer
        :param env - simpy.Environment
        :param order_generator - the store of orders
        :param workers: list of workers to be co-worked with
        :param id : unique id number of worker
        :param params: dict of params from yaml configuration file
    """

    def __init__(self, env: simpy.Environment, order_generator, workers: List[Worker], id: int, params: dict):
        self.env = env
        self.pose = np.array([0., 0.])
        self.goal = np.array([0., 0.])
        self.mean_velocity = np.array(params["robot_speed"]) * 60  # meters/minute

        self.order_generator = order_generator
        self.work_process = env.process(self.take_order())
        self.orders_done = 0
        self.workers = workers
        self.idle = []
        self.going_to_worker = []
        self.going_to_base = []
        self.idle_start = 0
        self.id = id
        self.params = params

    def take_order(self):
        """ process yielding the new available orders, when the robot is free"""

        while 1:
            # self.idle_start = self.env.now
            order = yield self.order_generator.store.get()
            # yield self.env.process(self.find_worker(order))
            yield self.env.process(self.find_smart_worker(order))
            # if self.idle_start is not None:

            # find nearest person
            # find meeting point (done?)

    def find_smart_worker(self, order: Order):
        """

        :param order: object of order class, used to calc distance and find meeting point (mp)
        :yield: timeout
        """
        nearest_worker = {"id": None, "time": np.inf, "mp": None}
        while nearest_worker["id"] is None:
            counter = 0
            for worker in self.workers:
                if worker.busy_until - self.env.now > 10:
                    continue
                counter += 1
                order_done_time, mp = worker.try_new_order(order, self)

                if order_done_time < nearest_worker["time"]:
                    nearest_worker["id"] = worker
                    nearest_worker["time"] = order_done_time
                    nearest_worker["mp"] = mp

            if nearest_worker["id"] is None or counter < self.params["num_workers"]/8:

                nearest_worker = {"id": None, "time": np.inf, "mp": None}
                self.idle.append(2)
                yield self.env.timeout(2)
                self.idle_start = self.env.now

        initial_distance = np.linalg.norm(worker.pose - order.address)
        mp_distance = np.linalg.norm(nearest_worker["mp"] - order.address)
        if mp_distance > 3000:
            print("mp_distance {mp_distance}, initial_distance {initial_distance}".format(
                initial_distance=initial_distance, mp_distance=mp_distance))
        worker = nearest_worker["id"]
        worker.add_new_order(robot=self, order=order, mp=nearest_worker["mp"],
                             new_busy_until=nearest_worker["time"])

        yield self.go_to_point(nearest_worker["mp"])

        self.pose = nearest_worker["mp"]

        yield self.go_to_point(np.array([0., 0]))

        self.pose = np.array([0., 0])
        nearest_worker = {"id": None, "time": np.inf, "mp": None}
        self.idle_start = self.env.now

    def find_worker(self, order: Order):
        """
        process finding new available persons & meeting with them
        :param order: new order to be delivered
        """
        nearest_worker = {"id": None, "distance": np.inf}
        while nearest_worker["id"] is None:

            for worker in self.workers:
                if worker.robot is None:
                    distance_to_order = np.linalg.norm(worker.pose - order.address)
                    if nearest_worker["distance"] > distance_to_order:
                        nearest_worker["id"] = worker
                        nearest_worker["distance"] = distance_to_order
            if nearest_worker["id"] is None:
                yield self.env.timeout(0.5)

        # for worker in self.workers:
        #     if worker.robot is None:

        worker = nearest_worker["id"]
        self.idle.append(self.env.now - self.idle_start)
        self.idle_start = None
        if LOGGING_LEVEL > 1:
            print("\t\t Robot{id}:new slave (id{slave_id}) found! {time:0.2f}".format(id=self.id, slave_id=worker.id,
                                                                                      time=self.env.now))

        point = get_best_mp(robot_pose=self.pose, worker_pose=worker.pose, delay_robot=0, delay_person=0,
                            robot_speed=self.mean_velocity, worker_speed=worker.mean_velocity, max_time=100,
                            goal=order.address)

        worker.goal = np.array(point)
        worker.robot = self
        worker.order = order
        # going to meeting point
        self.going_to_worker.append(self.go_to_point(point))
        yield self.go_to_point(point)

        # going to base
        self.going_to_base.append(self.go_to_point(np.array([0, 0])))
        yield self.go_to_point(np.array([0, 0]))

        self.pose = np.array([0, 0])
        if LOGGING_LEVEL > 1:
            print("\t\t Robot{id}:done {time:0.2f}".format(id=self.id, time=self.env.now))

    def go_to_point(self, meet_point: np.array):
        """
        generator yielding timeout for moving from one point to other
        :param meet_point:
        """
        self.goal = meet_point
        time_to_goal = np.linalg.norm(self.goal - self.pose) / self.mean_velocity

        return self.env.timeout(time_to_goal)
