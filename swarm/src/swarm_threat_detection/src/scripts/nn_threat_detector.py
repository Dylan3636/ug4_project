# from basic_threat_detection import BasicThreatDetector
import numpy as np
import scipy
import rospy


class RNNModel:

    def __init__(self, model, xnormalizer, ynormalizer, seq_length=10, sigma=5, p_value=0.05, graph=None):
        self.model = model
        self.xnormalizer = xnormalizer
        self.ynormalizer = ynormalizer
        self.seq_length = seq_length
        self.cov = np.diag([sigma**2, sigma**2])
        self.graph = graph

    def predict(self, batch_history_msg):
        positions = []
        latest_headings = []
        latest_speeds = []
        ids = []
        for intruder_history_msg in batch_history_msg.batch_previous_states:
            vessel_positions = [[2*state_msg.x/1000.0, 2*state_msg.y/1000.0] for state_msg in intruder_history_msg.previous_states]
            if len(vessel_positions)<self.seq_length:
                padding = (self.seq_length-len(vessel_positions))*[vessel_positions[0]]
                padding.extend(vessel_positions)
                vessel_positions = padding
            positions.append(vessel_positions[::-1])
            latest_headings.append(intruder_history_msg.previous_states[0].heading)
            latest_speeds.append(intruder_history_msg.previous_states[0].speed)
            ids.append(intruder_history_msg.previous_states[0].sim_id)

        #positions = [[[state_msg.x, state_msg.y] for state_msg in intruder_history_msg] for intruder_history_msg in batch_history_msg]

        X = self.xnormalizer.transform(np.reshape(positions, [-1, 2]))
        if self.seq_length>1:
            X = X.reshape([-1, self.seq_length, 2])
        with self.graph.as_default():
            y_pred = self.ynormalizer.inverse_transform(self.model.predict(X))
        pred_speeds = np.sqrt(y_pred[:, 0]**2+y_pred[:, 1]**2)*10
        pred_headings = np.arctan2(y_pred[:, 1], y_pred[:, 0])
        delta_speed = pred_speeds-np.array(latest_speeds)
        delta_heading = pred_headings-np.array(latest_headings)
        rospy.logerr(y_pred)
        rospy.logerr(np.rad2deg(pred_headings))
        rospy.logerr(np.rad2deg(latest_headings))
        rospy.logerr(pred_speeds)
        rospy.logerr(latest_speeds)
        return delta_speed, delta_heading, ids
