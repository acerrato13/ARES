import numpy as np

class DynamicRoleSwitching2D:
    """
    2D version of dynamic role-switching potential function (G_O).
    Used for competitive drone racing or autonomous vehicle pursuit/avoidance in 2D.
    """

    def __init__(self, alpha=1.0, beta=4.0, gamma=5.0, delta_1=-0.5, delta_2=-1.0):
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta_1 = delta_1
        self.delta_2 = delta_2

    @staticmethod
    def path_function(theta):
        """
        Compute 2D path position at parameter theta.
        r(theta) = [6*sin(theta), 3*sin(2*theta)]
        """
        return np.array([6 * np.sin(theta), 3 * np.sin(2 * theta)])

    def compute_theta_delta(self, theta_op, theta_d):
        """Difference in path parameter (who’s ahead)."""
        return theta_op - theta_d

    def compute_deviation_difference(self, p_d, theta_d, p_op, theta_op):
        """
        Compute the deviation difference R between ego and opponent drones in 2D.
        R = ||(p_op - r(theta_op)) - (p_d - r(theta_d))||
        """
        r_d = self.path_function(theta_d)
        r_op = self.path_function(theta_op)

        deviation_d = p_d - r_d
        deviation_op = p_op - r_op
        deviation_diff = deviation_op - deviation_d

        return np.linalg.norm(deviation_diff)

    def compute_G_O(self, p_d, theta_d, p_op, theta_op):
        """Compute the dynamic role-switching potential G_O."""
        theta_delta = self.compute_theta_delta(theta_op, theta_d)
        R = self.compute_deviation_difference(p_d, theta_d, p_op, theta_op)

        gaussian_term = np.exp(-((theta_delta - self.delta_1) / self.alpha) ** 2)
        tanh_term = np.tanh(theta_delta - self.delta_2) / self.beta
        deviation_term = 1.0 / (1.0 + self.gamma * R ** 2)

        return gaussian_term * tanh_term * deviation_term

    def compute_G_O_with_details(self, p_d, theta_d, p_op, theta_op):
        """Compute G_O and show intermediate values (for debugging)."""
        theta_delta = self.compute_theta_delta(theta_op, theta_d)
        R = self.compute_deviation_difference(p_d, theta_d, p_op, theta_op)

        gaussian_term = np.exp(-((theta_delta - self.delta_1) / self.alpha) ** 2)
        tanh_term = np.tanh(theta_delta - self.delta_2) / self.beta
        deviation_term = 1.0 / (1.0 + self.gamma * R ** 2)
        G_O = gaussian_term * tanh_term * deviation_term

        mode = "Overtaking" if theta_delta > 0 else "Obstructing"

        return {
            'G_O': G_O,
            'theta_delta': theta_delta,
            'R': R,
            'gaussian_term': gaussian_term,
            'tanh_term': tanh_term,
            'deviation_term': deviation_term,
            'mode': mode
        }
