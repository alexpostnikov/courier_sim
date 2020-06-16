from typing import List, Tuple

import matplotlib.pyplot as plt
import plotly.graph_objects as go


def matplotlib_plot(num_r_data: List[float], num_w_data: List[float], robot_idle_data: List[float],
                    workers_idle_data: List[float], productivity_data: List[float]) -> Tuple[plt.Axes]:
    """

    :param num_r_data: x axis, List of robot numbers per simulation
    :param num_w_data: y axis, List of workers numbers per simulation
    :param robot_idle_data: z axis, List of robots idle times per simulation
    :param workers_idle_data: z axis, List of workers idle times per simulation
    :param productivity_data: z axis, List of productivity per simulation
    :return: matplotlib axes with plotted productivity and idle times
    """
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
    return ax0, ax1


def plotly_plot(num_r_data: List[float], num_w_data: List[float], robot_idle_data: List[float],
                workers_idle_data: List[float], productivity_data: List[float]):
    """
    :param num_r_data: x axis, List of robot numbers per simulation
    :param num_w_data: y axis, List of workers numbers per simulation
    :param robot_idle_data: z axis, List of robots idle times per simulation
    :param workers_idle_data: z axis, List of workers idle times per simulation
    :param productivity_data: z axis, List of productivity per simulation
    :return: two figures with plotted productivity and idle times
    """

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
    return fig, fig2
