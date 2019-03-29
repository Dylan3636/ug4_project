import numpy as np

class CosineAnnealingWithWarmRestarts(object):
    """Cosine annealing scheduler, implemented as in https://arxiv.org/pdf/1608.03983.pdf"""

    def __init__(self,
                 min_learning_rate=0.00001,
                 max_learning_rate=0.01,
                 total_iters_per_period=100,
                 max_learning_rate_discount_factor=0.9,
                 period_iteration_expansion_factor=1):
        """
        Instantiates a new cosine annealing with warm restarts learning rate scheduler
        :param min_learning_rate: The minimum learning rate the scheduler can assign
        :param max_learning_rate: The maximum learning rate the scheduler can assign
        :param total_epochs_per_period: The number of epochs in a period
        :param max_learning_rate_discount_factor: The rate of discount for the maximum learning rate after each restart i.e. how many times smaller the max learning rate will be after a restart compared to the previous one
        :param period_iteration_expansion_factor: The rate of expansion of the period epochs. e.g. if it's set to 1 then all periods have the same number of epochs, if it's larger than 1 then each subsequent period will have more epochs and vice versa.
        """
        self.min_learning_rate = min_learning_rate
        self.max_learning_rate = max_learning_rate
        self.total_epochs_per_period = total_iters_per_period

        self.max_learning_rate_discount_factor = max_learning_rate_discount_factor
        self.period_iteration_expansion_factor = period_iteration_expansion_factor


    def update_learning_rule(self, epoch_number, learning_rate):
        """Update the hyperparameters of the learning rule.

        Run at the beginning of each epoch.

        Args:
            learning_rule: Learning rule object being used in training run,
                any scheduled hyperparameters to be altered should be
                attributes of this object.
            epoch_number: Integer index of training epoch about to be run.
        """
        print("epoch")
        if self.period_iteration_expansion_factor != 1:
            n = np.floor(np.log(1+(self.period_iteration_expansion_factor-1)*(epoch_number/self.total_epochs_per_period))/np.log(self.period_iteration_expansion_factor))

            total_epochs_per_period = ((1-self.period_iteration_expansion_factor**(n+1))/(1-self.period_iteration_expansion_factor))*self.total_epochs_per_period

        else:
            n = np.floor(epoch_number/self.total_epochs_per_period)
            total_epochs_per_period = self.total_epochs_per_period*(self.period_iteration_expansion_factor)**n

        discount = self.max_learning_rate_discount_factor**n
        epoch_number = epoch_number%total_epochs_per_period

        cos_val = np.cos(np.pi*(epoch_number/total_epochs_per_period))
        lr_diff = discount*self.max_learning_rate - self.min_learning_rate
        learning_rate = self.min_learning_rate + 0.5*lr_diff*(1+cos_val)

        return learning_rate


