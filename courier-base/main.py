from random import seed, randint
from typing import List

import numpy as np
import simpy

LOGGING_LEVEL = 0
seed(1)


### time measuremets in minutes
### distance measuremets in meters

class Order_generator:

    def __init__(self, env: simpy.Environment):
        self.env = env
        self.store = simpy.Store(env, capacity=1000000)
        self.store_gen_proc = self.env.process(self.gen_proc())
        self.order_counter = 0

    def gen_proc(self):
        while True:
            t = randint(5, 10)
            yield self.env.timeout(t)
            self.order_counter += 1
            order = Order(self.env, self.order_counter)
            self.store.put(order)
            if LOGGING_LEVEL > 1:
                print ("\t Order_generator: orders in queue:", len(self.store.items))

    def get_order(self):
        pass


class Order:
    """
    :param env: the simpy environment
    """
    def __init__(self, env: simpy.Environment, id: int):
        self.env = env
        self.address = np.array([randint(-3000, 3000), randint(-3000, 3000)])
        self.executor = None
        self.started = self.env.event()
        self.ended = self.env.event()
        self.order_ctrl_proc = env.process(self.order_ctrl())
        self.id = id

    def order_ctrl(self):
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

    def __init__(self, env: simpy.Environment, order_generator):
        self.env = env
        self.pose = np.array([0., 0.])
        self.goal = np.array([0., 0.])
        self.mean_velocity = np.array([180.])  # meters/minute

        self.order_generator = order_generator
        self.work_process = env.process (self.take_order())
        self.orders_done = 0

    def take_order(self):
        while 1:
            order = yield self.order_generator.store.get()
            self.goal = order.address
            order.started.succeed()
            time_to_goal = np.linalg.norm(self.goal-self.pose)/self.mean_velocity

            yield self.env.timeout(time_to_goal)
            self.pose = self.goal
            order.ended.succeed()
            # print ("going home", env.now)
            yield self.env.timeout(time_to_goal)
            self.pose = np.array([0., 0.])
            if LOGGING_LEVEL > 2:
                print("\t\tWorker: at home home", env.now)
            self.orders_done +=1


class Monitor:
    def __init__(self, env: simpy.Environment, workers: List[Worker]):
        self.env = env
        self.workers = workers
        self.order_ctrl_proc = env.process(self.order_ctrl())

    def order_ctrl(self):
        while 1:
            yield self.env.timeout(60)
            total = sum([worker.orders_done for worker in self.workers])
            print("\t Monitor: mean productivity (orders per hour) {prod:.2f}: ".format(prod=total / env.now*60) )


# class Robot:
#     def __init__(self, env: simpy.Environment, order_generator):
#         self.env = env
#         self.pose = np.array([0., 0.])
#         self.goal = np.array([0., 0.])
#         self.mean_velocity = np.array([180.])  # meters/minute
#
#         self.order_generator = order_generator
#         self.work_process = env.process(self.take_order())
#         self.orders_done = 0


if __name__ == "__main__":
    env = simpy.Environment()
    gen = Order_generator(env)
    workers = [Worker(env, gen) for i in range(50)]
    m = Monitor(env, workers)

    # res = simpy.Resource(env, capacity=1)
    # order = Order(env)


    # env.process(clock(env, 'fast', 0.5))
    # env.process(clock(env, 'slow', 1))

    # school = School(env)
    env.run(until=12000.1)


