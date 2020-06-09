import simpy
import yaml

from courier_base.courier_base import Robot, Order_generator, Worker, Monitor

if __name__ == "__main__":

    with open("./configs/params.yaml", 'r') as fp:
        params = yaml.load(fp, Loader=yaml.FullLoader)
    env = simpy.Environment()
    gen = Order_generator(env, params)
    workers = [Worker(env, gen, i, params) for i in range(params["num_workers"])]
    m = Monitor(env, workers, params)
    r = [Robot(env, gen, workers, i, params) for i in range(params["num_robots"])]
    env.run(until=params["sim_time"])


