from random import seed, randint
from typing import List

import numpy as np
import simpy
import yaml

LOGGING_LEVEL = 0
seed(1)
np.random.seed(1)

# TODO: find clothest person (+ check when they will freed)
# TODO: find optimal meeting point? (depend on both positions, fraction(workers/robots), order address)

### time measuremets in minutes
### distance measuremets in meters

class Order_generator:
    """
    generation of orders to be performed
    :param env:  - simpy.Environment
    :param params: dict of params from yaml configuration file
    """

    def __init__(self, env: simpy.Environment, params: dict):
        self.env = env
        self.store = simpy.Store(env, capacity=1000000)
        self.store_gen_proc = self.env.process(self.gen_proc())
        self.order_counter = 0
        self.params = params

    def gen_proc(self):
        """
        process - generator yieldin new orders given random (bounded) frequency
        """
        while True:
            t = max(np.random.normal(params["orders"]["mean"], params["orders"]["var"]), 0.1)
            yield self.env.timeout(t)
            self.order_counter += 1
            order = Order(self.env, self.order_counter, self.params)
            self.store.put(order)
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
                print('\t\t Order {id}: order. created at {time}'.format(id=self.id, time=env.now) )
            yield self.started
            if LOGGING_LEVEL > 2:
                print("\t\t Order {id}: order started at {time}".format(id=self.id, time=env.now))

            yield self.ended
            if LOGGING_LEVEL > 2:
                print("\t\t Order {id}: done at {time}".format(id=self.id, time=env.now))
            break


class Worker:
    """
    The person (worker) that bring orders to customer
    :param env - simpy.Environment
    :param id : unique id number of worker
    :param params: dict of params from yaml configuration file
    """

    def __init__(self, env: simpy.Environment, order_generator, id:int, params: dict):
        self.env = env
        self.pose = np.array([0., 0.])
        self.goal = np.array([0., 0.])
        self.mean_velocity = np.array(params["workers_speed"])  # meters/minute

        self.order_generator = order_generator
        # self.work_process = env.process (self.take_order())
        # self.obey_process = env.process(self.obey_robot(None))
        self.obey_robot_proc = self.env.event()
        self.new_work_reactivate = env.event()
        self.orders_done = 0
        self.resourse = simpy.Resource(self.env, capacity=1)
        self.obey_proc = self.env.process(self.obey_robot())
        self.request = None
        self.robot = None
        self.order = None
        self.id = id

    # if working without robots
    def take_order(self):
        """
         start new order processing (solo policy) if previous task is done
        """
        while 1:
            order = yield self.order_generator.store.get()
            self.goal = order.address
            time_to_goal = np.linalg.norm(self.goal-self.pose)/self.mean_velocity

            yield self.env.timeout(time_to_goal)
            self.pose = self.goal
            order.ended.succeed()

            yield self.env.timeout(time_to_goal)
            self.pose = np.array([0., 0.])
            if LOGGING_LEVEL > 2:
                print("\t\tWorker: at home home", env.now)
            self.orders_done += 1


    #if working with robots
    def obey_robot(self):
        """
        start new order processing (robot policy) if previous task is done
        """
        while 1:
            if self.robot is not None:

                print("\t\tWorker: obeying!", env.now)
                self.request = self.resourse.request()

                yield self.go_to_point(self.goal)
                self.pose = self.goal
                yield self.go_to_point(self.order.address)
                self.pose = self.order.address
                self.order = None
                self.resourse.release(self.request)
                print("\t\tWorker: it was pleasure!", env.now)
                self.robot = None
                self.orders_done += 1

            else:
                yield self.env.timeout(1)

    def go_to_point(self, meet_point: np.array):
        """
        generator yielding timeout for moving from one point to other
        :param meet_point:
        """

        self.goal = meet_point
        time_to_goal = np.linalg.norm(self.goal - self.pose) / self.mean_velocity
        return self.env.timeout(time_to_goal)


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
            total = sum([worker.orders_done for worker in self.workers])
            print("\t Monitor: mean productivity (orders per hour) {prod:.2f}: ".format(prod=total / env.now*60))


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
        self.mean_velocity = np.array(params["robot_speed"])  # meters/minute

        self.order_generator = order_generator
        self.work_process = env.process(self.take_order())
        self.orders_done = 0
        self.workers = workers
        self.id = id


    def take_order(self):
        """ process yielding the new available orders, when the robot is free"""

        while 1:
            order = yield self.order_generator.store.get()
            yield self.env.process(self.find_worker(order))
            # find nearest person
            # find meeting point
            # set new goals (for both of them)

    def find_worker(self, order: Order):
        """
        process finding new available persons & meeting with them
        :param order: new order to be delivered
        """

        while 1:
            yield self.env.timeout(randint(0, 100) / 100)
            for worker in self.workers:
                # if worker.resourse.capacity - worker.resourse.count > 0:
                if worker.robot is None:
                    print("\t\t Robot{id}:new slave found! {time}".format(id = self.id, time=self.env.now))
                    point = (worker.pose + self.pose) / 2
                    worker.goal = np.array(point)
                    worker.robot = self
                    worker.order = order

                    yield self.go_to_point(point)

                    yield self.go_to_point(np.array([0, 0]))
                    self.pose = np.array([0, 0])
                    print("\t\t Robot{id}:done {time}".format(id=self.id, time=self.env.now))

    def go_to_point(self, meet_point: np.array):
        """
        generator yielding timeout for moving from one point to other
        :param meet_point:
        """
        self.goal = meet_point
        time_to_goal = np.linalg.norm(self.goal - self.pose) / self.mean_velocity

        return self.env.timeout(time_to_goal)


if __name__ == "__main__":

    with open("params.yaml", 'r') as fp:
        params = yaml.load(fp, Loader=yaml.FullLoader)
    env = simpy.Environment()
    gen = Order_generator(env, params)
    workers = [Worker(env, gen, i, params) for i in range(params["num_workers"])]
    m = Monitor(env, workers, params)
    r = [Robot(env, gen, workers, i, params) for i in range(params["num_robots"])]

    # res = simpy.Resource(env, capacity=1)
    # order = Order(env)

    env.run(until=params["sim_time"])


