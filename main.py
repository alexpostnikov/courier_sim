from typing import Dict, Tuple

import simpy
import yaml

from courier_base.courier_base import Robot, OrderGenerator, Worker, Monitor
from plotting.plot_utils import plotly_plot


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
        for num_r in [10, 20, 30, 40, 50]:
            for num_w in [20, 30, 40]:
                params["num_workers"] = num_w
                params["num_robots"] = num_r
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
        for num_w in [40, 50, 60, 70]:
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

    fig, fig2 = plotly_plot(num_r_data, num_w_data, robot_idle_data, workers_idle_data, productivity_data)
    fig.show()
    fig2.show()


if __name__ == "__main__":
    run()