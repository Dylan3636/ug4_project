import numpy as np
import rospy
from threading import Lock

delta_time_secs = rospy.get_param('swarm_simulation/delta_time_secs')
lock = Lock()


class IntruderThreat:
    def __init__(self, is_threat, threat_evidence):
        self.is_threat = is_threat
        self.threat_evidence = threat_evidence


class BasicThreatDetector:

    def __init__(self, p_alert=0.6, learning_rate=0.1, max_dist_to_observe_threat=1000, noise_sigma=0.05):
        self.intruders = {}
        self.lr = learning_rate
        self.max_dist_to_observe_threat = max_dist_to_observe_threat
        self.p_alert = p_alert
        self.noise_sigma = noise_sigma

    def update_detector(self,
                        intruder_id,
                        dist_to_intruder):

        with lock:
            if intruder_id not in self.intruders:
                is_threat =\
                    [param['is_threat'] for param in rospy.get_param('swarm_simulation/intruder_params/')
                     if param['sim_id'] == intruder_id][0]
                self.intruders[intruder_id] = IntruderThreat(is_threat, 0.0)
            if dist_to_intruder < self.max_dist_to_observe_threat:
                noise = self.noise_sigma*np.random.randn()  # (0.1*self.intruders[intruder_id].threat_evidence)
                self.intruders[intruder_id].threat_evidence += \
                    delta_time_secs*self.lr*((1-dist_to_intruder/self.max_dist_to_observe_threat) + noise)
                self.intruders[intruder_id].threat_evidence = \
                    max(min(self.intruders[intruder_id].threat_evidence, 1), 0)
            if self.intruders[intruder_id].is_threat:
                if self.intruders[intruder_id].threat_evidence > 0.0:
                    threat_prob = self.intruders[intruder_id].threat_evidence
                else:
                    threat_prob = 0.05
            else:
                if self.intruders[intruder_id].threat_evidence > 0.0:
                    threat_prob = 1-self.intruders[intruder_id].threat_evidence
                else:
                    threat_prob = 0.05

        return (threat_prob > self.p_alert), threat_prob
