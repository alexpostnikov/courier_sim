import simpy
import yaml
import matplotlib.pyplot as plt
from courier_base.courier_base import Robot, OrderGenerator, Worker, Monitor
from typing import  Dict, Tuple


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


def autolabel(rects):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        ax.annotate('{value:0.1f}'.format(value=height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')


if __name__ == "__main__":

    # for fp  in range(6):
    #     path = "./configs/params_"+str(fp)+".yaml"
    #
    #
    #     with open(path, 'r') as fp:
    #         params = yaml.load(fp, Loader=yaml.FullLoader)
    #
    #     robot_idle, worker_idle, productivity = simulate(params)
    #
    #
    #
    #     fig, ax = plt.subplots()
    #     names = ["robot idle", "worker idle",  "productivity"]
    #     robot_idle_precents = robot_idle/params["sim_time"]/params["num_robots"]
    #     worker_idle_precents = worker_idle/params["sim_time"]/params["num_workers"]
    #     values = [robot_idle_precents, worker_idle_precents, productivity]
    #     rects1 = ax.bar(names, values)
    #     plt.bar(names, values)
    #     ax.set_title("workers: {n_w} , robots: {n_r}, new orders per hour {freq}".format(n_w=params["num_workers"], n_r=params["num_robots"], freq=60. / params["orders"]["mean"]))
    #     autolabel(rects1)
    #
    # plt.show()

    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

    import matplotlib.pyplot as plt
    import numpy as np

    productivity_data = []
    robot_idle_data = []
    workers_idle_data = []
    num_w_data = []
    num_r_data = []
    with open("./configs/params_0.yaml", 'r') as fp:
        params = yaml.load(fp, Loader=yaml.FullLoader)
        # robot_idle, worker_idle, productivity = simulate(params)

        for num_w, num_r in [[50, 20], [50, 30], [50, 40], [50, 50],
                             [60, 20], [60, 30], [60, 40], [60, 50],
                             [70, 20], [70, 30], [70, 40], [70, 50],
                             [40, 20], [40, 30], [40, 40], [40, 50],
                             ]:
            params["num_workers"] = num_w
            params["num_robots"] = num_r
            robot_idle, worker_idle, productivity = simulate(params)
            robot_idle_precents = robot_idle / params["sim_time"] / params["num_robots"]
            worker_idle_precents = worker_idle / params["sim_time"] / params["num_workers"]
            robot_idle_data.append(robot_idle_precents)
            workers_idle_data.append(worker_idle_precents)
            productivity_data.append(productivity / params["num_workers"])
            num_w_data.append(num_w)
            num_r_data.append(num_r)

    # SOLO
    with open("./configs/params_solo.yaml", 'r') as fp:
        params = yaml.load(fp, Loader=yaml.FullLoader)
        for num_w in [40, 50, 60, 70]:
            params["num_workers"] = num_w
            robot_idle, worker_idle, productivity = simulate(params)
            worker_idle_precents = worker_idle / params["sim_time"] / params["num_workers"]
            productivity_data.append(productivity / params["num_workers"])
            robot_idle_data.append(0)
            workers_idle_data.append(worker_idle_precents)
            num_w_data.append(num_w)
            num_r_data.append(0)

        print("solo productivity: {productivity}:0.2f, num workers: {workers}".format(productivity=productivity,
                                                                                      workers=params["num_workers"]))

    fig = plt.figure()
    ax0 = fig.add_subplot(211, projection='3d')
    ax1 = fig.add_subplot(212, projection='3d')
    ax0.scatter(num_r_data, num_w_data, productivity_data)
    ax1.scatter(num_r_data, num_w_data, robot_idle_data, marker="o", label="robot idle")
    ax1.scatter(num_r_data, num_w_data, workers_idle_data, marker="^", label="worker idle")

    ax0.set_xlabel('number of robots')
    ax0.set_ylabel('number of workers')
    ax0.set_zlabel('Z Label')

    ax1.set_xlabel('number of robots')
    ax1.set_ylabel('number of workers')
    ax1.set_zlabel('idle time')
    ax1.legend()
    plt.show()

    import plotly.express as px
    import plotly.graph_objects as go
    from plotly.graph_objs import Layout
    from plotly.subplots import make_subplots

    # fig = make_subplots(
    #     rows=1, cols=2,
    #     specs=[[{'type': 'scene'}, {'type': 'scene'}]])

    fig = go.Figure(data=[go.Scatter3d(x=num_r_data, y=num_w_data, z=robot_idle_data,
                                       mode='markers', name="robot_idle_data", marker=dict(
            size=8,
            opacity=0.8
        ))])

    fig.add_trace(go.Scatter3d(x=num_r_data, y=num_w_data, z=workers_idle_data, name="workers idle percentage ",
                               mode='markers', marker=dict(
            size=8,
            opacity=0.8,
        )))

    fig2 = go.Figure(data=[go.Scatter3d(x=num_r_data, y=num_w_data, z=productivity_data, name="productivity",
                                        mode='markers', marker=dict(
            size=8,
            opacity=0.8,
        ))])

    fig2.update_layout(scene=dict(
        xaxis_title='number of robots',
        yaxis_title='number of workers',
        zaxis_title='productivity time'),
    )

    fig.add_trace(go.Scatter3d(x=num_r_data, y=num_w_data, z=productivity_data, name="productivity_data ",
                               mode='markers', marker=dict(
            size=8,
            opacity=0.8
        )
                               ))

    fig.update_layout(scene=dict(
        xaxis_title='number of robots',
        yaxis_title='number of workers',
        zaxis_title='idle time & productivity per worker'),

        # width=700,
        # margin=dict(r=20, b=10, l=10, t=10
    )
    fig.show()
    fig2.show()

    # fig = px.scatter_3d([num_r_data, num_w_data, robot_idle_data], x='number of robots', y='number of workers', z='robot_idle',
    #                     color='petal_length', symbol='species')
    # fig.show()
    #
    # x2, y2 = np.meshgrid(num_r_data, num_w_data)
    # from scipy.interpolate import griddata
    # Z = griddata(productivity_data, (x2, y2), method='cubic')
    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # surf = ax.plot_surface(x2, y2, Z, rstride=1, cstride=1)
