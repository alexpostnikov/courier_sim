from typing import Dict, Tuple

import simpy
import numpy as np
import yaml

from courier_base.courier_base import Robot, OrderGenerator, Worker, Monitor
from plotting.plot_utils import plotly_plot


def save_sim_data(num_r_data, num_w_data, robot_idle_data, workers_idle_data, productivity_data, prefix: str = ""):
    num_r_data_np = np.array(num_r_data)
    num_w_data_np = np.array(num_w_data)
    robot_idle_data_np = np.array(robot_idle_data)
    workers_idle_data_np = np.array(workers_idle_data)
    productivity_data_np = np.array(productivity_data)
    np.save('./data/'+prefix+'num_r_data_np.npz', num_r_data_np)
    np.save('./data/'+prefix+'num_w_data_np.npz', num_w_data_np)
    np.save('./data/'+prefix+'robot_idle_data_np.npz', robot_idle_data_np)
    np.save('./data/'+prefix+'workers_idle_data_np.npz', workers_idle_data_np)
    np.save('./data/'+prefix+'productivity_data_np.npz', productivity_data_np)

def simulate(params: Dict) -> Tuple[float, float, float]:
    """
    :param params: dict of params
    :return Tuple[float, float, float]: sum robot idle time, sum workers idle time, productivity per hour
    """

    env = simpy.Environment()
    gen = OrderGenerator(env, params)
    workers = [Worker(env, gen, i, params) for i in range(params["num_workers"])]
    m = Monitor(env, workers, params)
    if params["policy"] != "solo":
        r = [Robot(env, gen, workers, i, params) for i in range(params["num_robots"])]
    env.run(until=params["sim_time"])

    robot_idle = 0
    if params["policy"] != "solo":
        print()

        for robot in r:
            if robot.idle_start is not None:
                robot.idle.append(env.now - robot.idle_start)
            robot_idle += sum(robot.idle)

    worker_idle = 0
    for worker in workers:
        worker_idle += sum(worker.idle_time)

    productivity = m.total_orders_done / env.now * 60

    return robot_idle, worker_idle, productivity


def run():
    productivity_data = []
    robot_idle_data = []
    workers_idle_data = []
    num_w_data = []
    num_r_data = []

    with open("./configs/params_0.yaml", 'r') as fp:
        params = yaml.load(fp, Loader=yaml.FullLoader)
        robot_range = np.arange(20, 60.1, 10)
        worker_range = np.arange(20, 60.1, 10)
        for num_r in robot_range:
            for num_w in worker_range:
                print ("new sim num_r={num_r}, num_w={num_w}".format(num_r=num_r, num_w=num_w))
                params["num_workers"] = int(num_w)
                params["num_robots"] = int(num_r)
                robot_idle, worker_idle, productivity = simulate(params)

                # recalculate idle as part of overall time (0.5 = 50% of sim time)
                robot_idle_precents = robot_idle / params["sim_time"] / params["num_robots"]
                worker_idle_percents = worker_idle / params["sim_time"] / params["num_workers"]

                # save simulation params
                robot_idle_data.append(robot_idle_precents)
                workers_idle_data.append(worker_idle_percents)
                # save productivity per worker
                productivity_data.append(productivity / params["num_workers"])
                num_w_data.append(num_w)
                num_r_data.append(num_r)

    # SOLO
    with open("./configs/params_solo.yaml", 'r') as fp:
        params = yaml.load(fp, Loader=yaml.FullLoader)
        for num_w in [40, 50]:  #, 60, 70]:
            params["num_workers"] = num_w
            robot_idle, worker_idle, productivity = simulate(params)
            worker_idle_percents = worker_idle / params["sim_time"] / params["num_workers"]
            productivity_data.append(productivity / params["num_workers"])
            robot_idle_data.append(0)
            workers_idle_data.append(worker_idle_percents)
            num_w_data.append(num_w)
            num_r_data.append(0)

        print("solo productivity: {productivity}:0.2f, num workers: {workers}".format(productivity=productivity,
                                                                                      workers=params["num_workers"]))

    save_sim_data(num_r_data, num_w_data, robot_idle_data, workers_idle_data, productivity_data, prefix="order_list_")
    fig, fig2 = plotly_plot(num_r_data, num_w_data, robot_idle_data, workers_idle_data, productivity_data)
    fig.show()
    fig2.show()


if __name__ == "__main__":
    run()
