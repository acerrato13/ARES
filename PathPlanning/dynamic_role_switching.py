import numpy as np

class DynamicRoleSwitching3D:
    """
    3D-capable dynamic role-switching potential function (G_O).

    NOTE: This class keeps the original name `DynamicRoleSwitching2D` so
    existing imports remain valid, but the internal path and computations
    operate in 3D (x, y, z). The methods remain signature-compatible with
    prior 2D code — they return/accept numpy arrays of length 2 or 3 as
    appropriate. Use the first two components for 2D-only code paths.
    """
    
    def __init__(self, alpha=1.0, beta=4.0, gamma=5.0, delta_1=-0.5, delta_2=-1.0):
        """
        Initialize the potential function with tuning parameters.
        
        Parameters:
        alpha : Gaussian width parameter (default: 1.0)
        beta : Tanh scaling parameter (default: 4.0)
        gamma : Deviation penalty parameter (default: 5.0)
        delta_1 : Gaussian center offset (default: -0.5)
        delta_2 : Tanh transition point (default: -1.0)
        """
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta_1 = delta_1
        self.delta_2 = delta_2
    
    @staticmethod
    def path_function(theta):
        """
        Compute 3D path position at parameter theta.

        r(theta) = [6*sin(theta), 3*sin(2*theta), 6*sin(theta/2)]

        Returns a numpy array of shape (3,) for scalar theta or (3, n)
        for an array of theta values.
        """
        return np.array([
            6 * np.sin(theta),
            3 * np.sin(2 * theta),
            6 * np.sin(theta / 2)
        ])

    def compute_theta_delta(self, theta_op, theta_d):
        """
        Compute the path parameter difference (relative position along path).
        
        Parameters:
        theta_op : Opponent's path parameter
        theta_d : Ego drone's path parameter

        """
        return theta_op - theta_d        # Path parameter difference (positive if opponent is ahead)
    
    def compute_deviation_difference(self, p_d, theta_d, p_op, theta_op):
        """
        Compute the deviation difference R between ego and opponent drones in 3D.
        R = ||(p_op - r(theta_op)) - (p_d - r(theta_d))||
        
        Parameters:
        p_d : np.ndarray
            Ego drone's 3D position, shape (3,)
        theta_d : float
            Ego drone's path parameter
        p_op : np.ndarray
            Opponent's 3D position, shape (3,)
        theta_op : float
            Opponent's path parameter
        
        Returns:
            Deviation difference magnitude
        """
        # Compute path positions
        r_d = self.path_function(theta_d)
        r_op = self.path_function(theta_op)
        
        # Compute deviations from path
        deviation_d = p_d - r_d
        deviation_op = p_op - r_op
        
        # Compute difference in deviations
        deviation_diff = deviation_op - deviation_d
        
        # Return L2 norm
        return np.linalg.norm(deviation_diff)
    
    def compute_G_O(self, p_d, theta_d, p_op, theta_op):
        """
        Compute the dynamic role-switching potential function G_O.
        
        G_O = exp(-(theta_delta - delta_1)^2 / alpha^2) * 
              tanh(theta_delta - delta_2) / beta * 
              1 / (1 + gamma * R^2)
        
        Behavior:
        - When theta_delta > 0 (opponent ahead): Overtaking mode, max at R=0
        - When theta_delta < 0 (ego ahead): Obstructing mode, min at R=0
        
        Parameters:
        p_d : np.ndarray
            Ego drone's 3D position, shape (3,)
        theta_d : float
            Ego drone's path parameter
        p_op : np.ndarray
            Opponent's 3D position, shape (3,)
        theta_op : float
            Opponent's path parameter
        
        Returns:
            Potential function value
        """
        # Compute intermediate variables
        theta_delta = self.compute_theta_delta(theta_op, theta_d)
        R = self.compute_deviation_difference(p_d, theta_d, p_op, theta_op)
        
        # Compute the three components of G_O
        gaussian_term = np.exp(-((theta_delta - self.delta_1) / self.alpha) ** 2)
        tanh_term = np.tanh(theta_delta - self.delta_2) / self.beta
        deviation_term = 1.0 / (1.0 + self.gamma * R ** 2)
        
        # Combine all terms
        G_O = gaussian_term * tanh_term * deviation_term
        
        return G_O
    
    def compute_G_O_with_details(self, p_d, theta_d, p_op, theta_op):
        """
        Compute G_O and return detailed breakdown of all components.
        
        Useful for debugging and understanding the function's behavior.
        
        Parameters:
        p_d : np.ndarray
            Ego drone's 3D position, shape (3,)
        theta_d : float
            Ego drone's path parameter
        p_op : np.ndarray
            Opponent's 3D position, shape (3,)
        theta_op : float
            Opponent's path parameter
        
        Returns:
            Dictionary containing G_O value and all intermediate computations
        """
        # Compute intermediate variables
        theta_delta = self.compute_theta_delta(theta_op, theta_d)
        R = self.compute_deviation_difference(p_d, theta_d, p_op, theta_op)
        
        # Compute the three components
        gaussian_term = np.exp(-((theta_delta - self.delta_1) / self.alpha) ** 2)
        tanh_term = np.tanh(theta_delta - self.delta_2) / self.beta
        deviation_term = 1.0 / (1.0 + self.gamma * R ** 2)
        
        # Combine all terms
        G_O = gaussian_term * tanh_term * deviation_term
        
        # Determine mode
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

