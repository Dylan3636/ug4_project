# from basic_threat_detection import BasicThreatDetector
import numpy as np
import scipy
import rospy
from time import time


def radnorm(theta):
    while theta<0:
        theta += 2*np.pi
    while theta > 2*np.pi:
        theta -= 2*np.pi
    return theta


class FFNNModel:
    def __init__(self, model, xnormalizer, ynormalizer, seq_length=10, sigma=0, p_value=0.05, graph=None):
        self.model = model
        self.xnormalizer = xnormalizer
        self.ynormalizer = ynormalizer
        self.seq_length = seq_length
        self.cov = np.diag([sigma**2, sigma**2])
        self.sigma = sigma
        self.graph = graph

    def predict(self, batch_history_msg):
        # rospy.logerr("HERE1")
        vessel_states=[]
        latest_speeds=[]
        latest_headings=[]
        ids=[]
        scale = 10/4
        for intruder_history_msg in batch_history_msg.batch_previous_states:
            intruder_history_msg = intruder_history_msg.previous_states
            speed = intruder_history_msg[1].speed
            heading = intruder_history_msg[1].heading
            vessel_state = [20*intruder_history_msg[0].x/1000, 20*intruder_history_msg[0].y/1000, speed*np.cos(heading)/scale, speed*np.sin(heading)/scale]
            # rospy.logerr(intruder_history_msg[0].sim_id)
            # rospy.logerr(vessel_state)
            vessel_states.append(vessel_state)
            latest_speeds.append(speed)
            latest_headings.append(heading)
            ids.append(intruder_history_msg[0].sim_id)

        X = np.reshape(vessel_states, [-1, 4])
        # rospy.logerr(X)
        with self.graph.as_default():
            t = time()
            # self.model.predict(X)
            #rospy.logerr("NN predict time {}".format(time()-t))
            y_pred = self.ynormalizer.inverse_transform(self.model.predict(X))
            y_pred += self.sigma*np.random.randn(*np.shape(y_pred))/4
        pred_speeds = np.sqrt(y_pred[:, 0]**2+y_pred[:, 1]**2)*scale
        # rospy.logerr(pred_speeds)
        pred_headings = np.array([radnorm(theta) for theta in np.arctan2(y_pred[:, 1], y_pred[:, 0])])
        delta_speed = pred_speeds-np.array(latest_speeds)
        delta_heading = pred_headings-np.array(latest_headings)
        # rospy.logerr("ypred {}".format(y_pred))
        return delta_speed, delta_heading, ids


class RNNModel:

    def __init__(self, model, xnormalizer, ynormalizer, seq_length=10, sigma=0, p_value=0.05, graph=None):
        self.model = model
        self.xnormalizer = xnormalizer
        self.ynormalizer = ynormalizer
        self.seq_length = seq_length
        self.cov = np.diag([sigma**2, sigma**2])
        self.sigma = sigma
        self.graph = graph

    def predict(self, batch_history_msg):
        positions = []
        latest_headings = []
        latest_speeds = []
        ids = []
        for intruder_history_msg in batch_history_msg.batch_previous_states:
            vessel_positions = [[20*state_msg.y/1000.0, 20*state_msg.x/1000.0] for state_msg in intruder_history_msg.previous_states]
            if len(vessel_positions)<self.seq_length:
                if len(vessel_positions)==0:
                    # print(intruder_history_msg)
                    # rospy.logerr(str(batch_history_msg))
                    continue

                padding = (self.seq_length-len(vessel_positions))*[vessel_positions[0]]
                padding.extend(vessel_positions)
                vessel_positions = padding
            positions.append(vessel_positions)
            latest_headings.append(intruder_history_msg.previous_states[0].heading)
            latest_speeds.append(intruder_history_msg.previous_states[0].speed)
            ids.append(intruder_history_msg.previous_states[0].sim_id)

        #positions = [[[state_msg.x, state_msg.y] for state_msg in intruder_history_msg] for intruder_history_msg in batch_history_msg]

        X = self.xnormalizer.transform(np.reshape(positions[::-1], [-1, 2]))
        scale = 50/4
        if self.seq_length>1:
            X = X.reshape([-1, self.seq_length, 2])
        with self.graph.as_default():
            t = time()
            # self.model.predict(X)
            #rospy.logerr("NN predict time {}".format(time()-t))
            y_pred = self.ynormalizer.inverse_transform(self.model.predict(X))
            y_pred += self.sigma*np.random.randn(*np.shape(y_pred))/4
        pred_speeds = np.sqrt(y_pred[:, 0]**2+y_pred[:, 1]**2)*scale
        pred_headings = np.array([radnorm(theta) for theta in np.arctan2(y_pred[:, 1], y_pred[:, 0])])
        delta_speed = pred_speeds-np.array(latest_speeds)
        delta_heading = pred_headings-np.array(latest_headings)
        # rospy.logerr(y_pred)
        # rospy.loginfo(np.rad2deg(pred_headings))
        # rospy.loginfo(np.rad2deg(latest_headings))
        # rospy.loginfo(np.rad2deg(delta_heading))
        # rospy.logerr(pred_speeds)
        # rospy.loginfo(latest_speeds)
        # rospy.loginfo(delta_speed)
        return delta_speed, delta_heading, ids
