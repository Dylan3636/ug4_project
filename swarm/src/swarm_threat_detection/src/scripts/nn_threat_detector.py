# from basic_threat_detection import BasicThreatDetector
import numpy as np
import scipy


class RNNModel:

    def __init__(self, model, xnormalizer, ynormalizer, seq_length=10, sigma=5, p_value=0.05):
        self.model = model
        self.xnormalizer = xnormalizer
        self.ynormalizer = ynormalizer
        self.seq_length = seq_length
        self.cov = np.diag([sigma**2, sigma**2])

    def predict(self, batch_history_msg):
        positions = []
        latest_headings = []
        latest_speeds = []
        ids = []
        for intruder_history_msg in batch_history_msg.batch_previous_states:
            positions.append([[state_msg.x, state_msg.y] for state_msg in intruder_history_msg.previous_states])
            latest_headings.append(intruder_history_msg.previous_states[-1].heading)
            latest_speeds.append(intruder_history_msg.previous_states[-1].speed)
            ids.append(intruder_history_msg.previous_states[-1].sim_id)

        positions = [[[state_msg.x, state_msg.y] for state_msg in intruder_history_msg] for intruder_history_msg in batch_history_msg]
        X = self.xnormalizer.transform(np.reshape(positions, [-1, self.seq_length, 2]))
        y_pred = self.ynormalizer.inverse_transform(self.model.predict(X))
        pred_speeds = y_pred[0]**2+y_pred[1]**2
        pred_headings = np.arctan2(y_pred[1], y_pred[0])
        delta_speed = pred_speeds-np.array(latest_speeds)
        delta_heading = pred_headings-np.array(latest_headings)
        return delta_speed, delta_heading, ids
