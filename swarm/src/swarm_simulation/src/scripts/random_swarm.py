#!/home/dylan/miniconda3/envs/mlp/bin/python3

import rospy
import numpy as np
from plot import LivePlot
from simulation_listener import SimulationNode
from sim_objects import Intruder, BasicUSV, Tanker
from numpy.random import RandomState
from swarm_msgs.msg import resetSystem
from time import time, sleep
from swarmais.aisdata import drop_lt, drop_gt, drop_cond
import pandas as pd
import os


class Sampler:
    def __init__(self, randomstate):
        self.randomstate = randomstate

    def sample(self, num_samples=1):
        pass


class ConstantSampler(Sampler):
    def __init__(self, randomstate, constant):
        self.constant = constant
        super().__init__(randomstate)

    def sample(self, num_samples=1):
        return float(self.constant)


class UniformSampler(Sampler):
    def __init__(self, randomstate, lower_bound, upper_bound):
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        super().__init__(randomstate)

    def sample(self, num_samples=1):
        return float(self.randomstate.uniform(self.lower_bound, self.upper_bound))


class NormalSampler(Sampler):
    def __init__(self, randomstate, mean, sigma):
        self.mean = mean
        self.sigma = sigma
        super().__init__(randomstate)

    def sample(self, num_samples=1):
        return float(self.randomstate.normal(self.mean, self.sigma))


class HybridSampler(Sampler):
    def __init__(self, randomstate, samplers):
        super().__init__(randomstate)
        self.samplers = samplers

    def sample(self, num_samples=1):
        sampler = self.randomstate.choice(self.samplers)
        return sampler.sample()


class InitialStateSampler:
    def __init__(self, x_sampler: Sampler,
                 y_sampler: Sampler,
                 speed_sampler: Sampler,
                 heading_sampler: Sampler):
        self.x_sampler = x_sampler
        self.y_sampler = y_sampler
        self.speed_sampler = speed_sampler
        self.heading_sampler = heading_sampler

    def sample(self, num_samples=1):
        x = self.x_sampler.sample(num_samples)
        y = self.y_sampler.sample(num_samples)
        speed = self.speed_sampler.sample(num_samples)
        heading = float(np.deg2rad(self.heading_sampler.sample(num_samples)))
        return x, y, speed, heading


class AISInititialStateSampler(Sampler):
    def __init__(self, randomstate, data):
        super().__init__(randomstate)
        self.data=data

    def sample(self, num_samples=1):
        subdata=drop_lt(self.data,'Xcoord', -1)
        subdata=drop_lt(subdata, 'Ycoord', -1)
        subdata=drop_gt(subdata, 'Xcoord', 1)
        subdata=drop_gt(subdata, 'Ycoord', 1)
        row = subdata.sample(1, random_state=self.randomstate)
        x = 1000*row.Xcoord.iloc[0]
        y = 1000*row.Ycoord.iloc[0]
        x_dot = row.SmoothedVectorXcoord.iloc[0]
        y_dot = row.SmoothedVectorYcoord.iloc[0]
        print(row)
        print(x,y,x_dot,y_dot)
        heading = np.arctan2(y_dot, x_dot)
        speed = np.sqrt(x_dot**2 + y_dot**2)
        x, y, speed, heading = map(float, [x, y, speed, heading])
        return x, y, speed, heading


class ConstraintSampler:
    def __init__(self, max_speed_sampler: Sampler,
                 max_delta_speed_sampler: Sampler,
                 max_delta_heading_sampler: Sampler):
        self.max_speed_sampler = max_speed_sampler
        self.max_delta_speed_sampler = max_delta_speed_sampler
        self.max_delta_heading_sampler = max_delta_heading_sampler

    def sample(self, num_samples=1):
        max_speed = self.max_speed_sampler.sample(num_samples)
        max_delta_heading = self.max_delta_heading_sampler.sample(num_samples)
        max_delta_heading = float(np.deg2rad(max_delta_heading))
        max_delta_speed = self.max_delta_speed_sampler.sample(num_samples)
        return max_speed, max_delta_speed, max_delta_heading


class RadarConstraintSampler:
    def __init__(self,
                 max_distance_sampler: Sampler,
                 max_angle_sampler: Sampler,
                 aggression_sampler: Sampler):
        self.max_distance_sampler = max_distance_sampler
        self.max_angle_sampler = max_angle_sampler
        self.aggression_sampler = aggression_sampler

    def sample(self, num_samples=1):
        max_distance = self.max_distance_sampler.sample(num_samples)
        max_angle = float(np.deg2rad(self.max_angle_sampler.sample(num_samples)))
        aggression = self.aggression_sampler.sample(num_samples)
        return max_distance, max_angle, aggression


class USVSampler:
    def __init__(self, initial_state_sampler, constraint_sampler, radar_constraint_sampler):
        print(initial_state_sampler)
        self.initial_state_sampler = initial_state_sampler
        self.constraint_sampler = constraint_sampler
        self.radar_constraint_sampler = radar_constraint_sampler

    def sample(self, sim_id, radius=50):
        init_state = self.initial_state_sampler.sample()
        constraints = self.constraint_sampler.sample()
        radar_params = self.radar_constraint_sampler.sample()
        return BasicUSV(sim_id, init_state, constraints, radius_buffer=radius), radar_params


class IntruderSampler:
    def __init__(self, initial_state_sampler: InitialStateSampler, constraint_sampler: ConstraintSampler, radar_constraint_sampler: RadarConstraintSampler):
        self.initial_state_sampler = initial_state_sampler
        self.constraint_sampler = constraint_sampler
        self.radar_constraint_sampler = radar_constraint_sampler

    def sample(self, sim_id, activate_time, radius=50):
        init_state = self.initial_state_sampler.sample()
        constraints = self.constraint_sampler.sample()
        radar_params = self.radar_constraint_sampler.sample()
        return Intruder(sim_id,
                        init_state,
                        constraints,
                        activate_time=activate_time,
                        radius_buffer=radius), radar_params


def get_sampler_from_config(config: dict, randomstate):
    assert('sampler' in config)

    if config['sampler'].lower() == 'constant':
        return ConstantSampler(randomstate, config['constant'])

    elif config['sampler'].lower() == 'uniform':
        return UniformSampler(randomstate, config['lower'], config['upper'])

    elif config['sampler'].lower() == 'normal':
        return NormalSampler(randomstate, config['mean'], config['sigma'])

    elif config['sampler'].lower() == 'hybrid':
        return HybridSampler(randomstate, [get_sampler_from_config(sampler_config, randomstate)
                                           for sampler_config in config['samplers']])
    else:
        raise NotImplementedError


def get_sampler_dict_from_config(keys, config: dict, randomstate):
    samplers_dict = {}
    for key in keys:
        assert(key in config)
        samplers_dict[key+'_sampler'] = get_sampler_from_config(config[key], randomstate)
    return samplers_dict


def get_init_state_sampler_from_config(config, randomstate):
    init_state_sampler_dict = get_sampler_dict_from_config(['x', 'y', 'speed', 'heading'],
                                                           config['initial_state'],
                                                           randomstate)
    return InitialStateSampler(init_state_sampler_dict['x_sampler'],
                               init_state_sampler_dict['y_sampler'],
                               init_state_sampler_dict['speed_sampler'],
                               init_state_sampler_dict['heading_sampler'])


def get_constraints_sampler_from_config(config, randomstate):
    constraints_sampler_dict = get_sampler_dict_from_config(['max_speed', 'max_delta_speed', 'max_delta_heading'],
                                                            config['constraints'],
                                                            randomstate)
    return ConstraintSampler(constraints_sampler_dict['max_speed_sampler'],
                             constraints_sampler_dict['max_delta_speed_sampler'],
                             constraints_sampler_dict['max_delta_heading_sampler']
                             )


def get_radar_constraints_sampler_from_config(config, randomstate):
    radar_constraints_sampler_dict = get_sampler_dict_from_config(['max_distance', 'max_angle', 'aggression'],
                                                                  config['radar_constraints'],
                                                                  randomstate)
    return RadarConstraintSampler(radar_constraints_sampler_dict['max_distance_sampler'],
                                  radar_constraints_sampler_dict['max_angle_sampler'],
                                  radar_constraints_sampler_dict['aggression_sampler'],
                                  )


def get_usv_sampler_from_config(config, randomstate):
    assert'usv_params' in config, print(config)

    init_state_sampler = get_init_state_sampler_from_config(config['usv_params'], randomstate)
    constraints_sampler = get_constraints_sampler_from_config(config['usv_params'], randomstate)
    radar_params_sampler = get_radar_constraints_sampler_from_config(config['usv_params'], randomstate)

    return USVSampler(init_state_sampler, constraints_sampler, radar_params_sampler)


def get_intruder_sampler_from_config(config, randomstate):
    assert 'intruder_params' in config, print(config)

    init_state_sampler = get_init_state_sampler_from_config(config['intruder_params'], randomstate)
    constraints_sampler = get_constraints_sampler_from_config(config['intruder_params'], randomstate)
    radar_params_sampler = get_radar_constraints_sampler_from_config(config['intruder_params'], randomstate)

    return IntruderSampler(init_state_sampler, constraints_sampler, radar_params_sampler)


def constraints_to_params(constraints):
    assert(len(constraints) == 3)
    config = dict()
    config['max_speed'] = constraints[0]
    config['max_delta_speed'] = constraints[1]
    config['max_delta_heading'] = float(np.rad2deg(constraints[2]))
    return config


def radar_constraints_to_params(radar_constraints):
    assert(len(radar_constraints) == 3)
    config = dict()
    config['max_distance'] = radar_constraints[0]
    config['max_angle'] = float(np.rad2deg(radar_constraints[1]))
    config['aggression'] = radar_constraints[2]
    return config


def usv_to_params(usv: BasicUSV, radar_constraints):
    config = dict()
    config['sim_id'] = usv.sim_id
    config['constraints'] = constraints_to_params(usv.constraints)
    config['radar_parameters'] = radar_constraints_to_params(radar_constraints)
    return config


def intruder_to_params(intruder: Intruder, radar_constraints, is_threat):
    config = dict()
    config['sim_id'] = intruder.sim_id
    config['constraints'] = constraints_to_params(intruder.constraints)
    config['radar_parameters'] = radar_constraints_to_params(radar_constraints)
    config['is_threat'] = is_threat
    return config


class SimulationSampler(Sampler):
    def __init__(self, randomstate: RandomState):
        super().__init__(randomstate)
        self.config = rospy.get_param('/random_simulation')
        self.usv_sampler = get_usv_sampler_from_config(self.config, self.randomstate)
        self.intruder_sampler = get_intruder_sampler_from_config(self.config, self.randomstate)
        self.num_usvs = None
        self.radius_buffer = self.config['radius_buffer']
        self.delta_time_secs = self.config['delta_time_secs']
        self.threshold = self.config['threshold']
        self.visualize = self.config['visualize']
        self.anim = LivePlot() if self.visualize else None
        self.reset_pub = rospy.Publisher('SystemReset', resetSystem, queue_size=100)
        self.data = pd.read_csv("/home/dylan/Github/ug4_project/data.csv")
        self.intruder_initstate_sampler = AISInititialStateSampler(randomstate, self.data)
        self.intruder_sampler.initial_state_sampler = self.intruder_initstate_sampler
        rospy.set_param('/swarm_simulation/delta_time_secs', self.delta_time_secs)

    def sample_usvs(self):
        usv_config = self.config['usv_params']
        min_num_usvs = usv_config['min_num_usvs']
        max_num_usvs = usv_config['max_num_usvs']
        usv_samples = self._sample_usvs(self.usv_sampler,
                                        min_num_usvs,
                                        max_num_usvs,
                                        self.randomstate,
                                        self.radius_buffer)
        self.num_usvs = len(usv_samples)
        return usv_samples

    def sample_intruders(self):
        assert self.num_usvs is not None, "USVs MUST BE SAMPLED FIRST"
        intruder_config = self.config['intruder_params']
        min_num_intruders = intruder_config['min_num_intruders']
        max_num_intruders = intruder_config['max_num_intruders']
        min_num_threats = intruder_config['min_num_threats']
        time_between_intruders_mean = intruder_config['time_between_intruders_mean']
        return self._sample_intruders(self.intruder_sampler,
                                      min_num_intruders,
                                      max_num_intruders,
                                      min_num_threats,
                                      self.num_usvs,
                                      self.randomstate,
                                      time_between_intruders_mean,
                                      self.radius_buffer
                                      )

    def sample_simulation(self):
        asset_params = rospy.get_param('random_simulation/asset_params/asset')
        asset_sim_id = 100
        asset_radius_buffer = asset_params['radius_buffer']
        x = asset_params['init_state']['x']
        y = asset_params['init_state']['y']
        heading = asset_params['init_state']['heading']
        speed = asset_params['init_state']['speed']
        asset_init_state = [x, y, speed, np.deg2rad(heading)]
        tanker = Tanker(sim_id=asset_sim_id,
                        initial_state=asset_init_state,
                        radius_buffer=asset_radius_buffer)
        usvs = self.sample_usvs()
        intruders = self.sample_intruders()

        node = SimulationNode([*intruders, *usvs, tanker],
                              use_gui=self.visualize,
                              anim=self.anim,
                              threshold=self.threshold,
                              delta_t=self.delta_time_secs,
                              initialize=False)
        return node

    def reset(self):
        os.system('rosparam delete /swarm_simulation/{}'.format('usv_params'))
        os.system('rosparam delete /swarm_simulation/{}'.format('intruder_params'))
        # rospy.delete_param('/swarm_simulation/usv_params')
        # rospy.delete_param('/swarm_simulation/intruder_params')
        rospy.loginfo("Sending Reset Message")
        self.num_usvs = None
        msg = resetSystem()
        rate = rospy.Rate(10)
        t = time()
        dt = time()-t
        while dt < 3:
            self.reset_pub.publish(msg)
            rate.sleep()
            dt = time()-t

    @staticmethod
    def _sample_usvs(usv_sampler: USVSampler, min_num_usvs, max_num_usvs, randomstate: RandomState, radius=50):
        # Sample how many usvs sim will have
        num_usvs = randomstate.random_integers(min_num_usvs, max_num_usvs)

        rospy.set_param('/swarm_simulation/num_usvs', int(num_usvs))

        usv_configs = []
        usvs = []
        for sim_id in range(1, num_usvs+1):

            # Sample usv
            usv, radar_params = usv_sampler.sample(sim_id, radius)
            usvs.append(usv)

            # Get config dictionary
            usv_configs.append(usv_to_params(usv, radar_params))

        # Set configs to ROS param server
        rospy.set_param('/swarm_simulation/usv_params', usv_configs)
        return usvs

    @staticmethod
    def _sample_intruders(intruder_sampler: IntruderSampler,
                          min_num_intruders,
                          max_num_intruders,
                          min_num_threats,
                          num_usvs,
                          randomstate: RandomState,
                          time_between_intruders_mean=5,
                          radius=50):
        # Sample how many intruders sim will have
        num_intruders = randomstate.random_integers(min_num_intruders, max_num_intruders)

        # Sample how many of the intruders will be threats
        max_num_threats = min(num_usvs, num_intruders)
        min_num_threats = min(min_num_threats, max_num_threats)
        num_threats = randomstate.random_integers(min_num_threats, max_num_threats)
        print(min_num_threats, max_num_threats, num_threats, num_usvs)

        # Sample time between intruders (secs)
        delta_times = randomstate.exponential(time_between_intruders_mean, num_intruders)
        activate_times = np.cumsum(delta_times)
        print(delta_times)

        # Get sim ids
        intruder_ids = [100 + j for j in range(1, num_intruders+1)]

        # Uniformly choose which will be threats
        threat_ids = randomstate.choice(intruder_ids, num_threats, replace=False)

        intruders = []
        intruder_configs = []
        for intruder_id, activate_time in zip(intruder_ids, activate_times):

            # Sample intruder
            intruder, radar_params = intruder_sampler.sample(intruder_id, activate_time, radius)
            intruders.append(intruder)

            # Get config dictionary
            intruder_configs.append(intruder_to_params(intruder, radar_params, intruder_id in threat_ids))
            print(intruder_id, intruder_id in threat_ids)

        # Set configs to ROS param server
        rospy.set_param('/swarm_simulation/intruder_params', intruder_configs)
        return intruders


if __name__ == "__main__":
    rospy.init_node("RandomSimulation", anonymous=True)
    SEED = rospy.get_param('/random_simulation/seed')
    rospy.logerr("SEED : {}".format(SEED))
    NUM_SIMS = rospy.get_param('/random_simulation/num_simulations')
    rs = np.random.RandomState(seed=SEED)
    simulation_sampler = SimulationSampler(rs)
    while not rospy.is_shutdown():
        for i in range(NUM_SIMS):
            print("STARTING SIMULATION {}".format(i))
            sim_node = simulation_sampler.sample_simulation()
            result = sim_node.begin()
            simulation_sampler.reset()
            sim_node.unregister()
