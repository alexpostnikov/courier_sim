import simpy
import yaml

from courier_base.courier_base import Robot, OrderGenerator, Worker, Monitor

if __name__ == "__main__":

    with open("./configs/params.yaml", 'r') as fp:
        params = yaml.load(fp, Loader=yaml.FullLoader)

    env = simpy.Environment()
    gen = OrderGenerator(env, params)
    workers = [Worker(env, gen, i, params) for i in range(params["num_workers"])]
    m = Monitor(env, workers, params)
    if params["policy"] != "solo":
        r = [Robot(env, gen, workers, i, params) for i in range(params["num_robots"])]
    env.run(until=params["sim_time"])


