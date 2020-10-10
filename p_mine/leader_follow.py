from go_to_points import *
from get_init import *


class LeaderFollower:
    def __init__(self):
        self.nav = GoToPoints()
        self.debug = self.nav.model()
        self.N = self.nav.N
        self.parameter = parameter()

    def leader_follower(self):
        waypoints = np.array([[1.5, 0.2], [0.8, 0.2]])  # Waypoints the leader moves to.
        close_enough = 0.03  # How close the leader must get to the waypoint to move to the next one.
        L = completeGL(5)
        d = 0.15
        # weights = d * np.ones((5, 5))

        ddiag = np.sqrt(3) * d
        # Weight matrix to control inter-agent distances
        weights_forward = np.array([
            [0, d, d, 2 * d, 2 * d],
            [d, 0, d, d, ddiag],
            [d, d, 0, ddiag, d],
            [2 * d, d, ddiag, 0, 2 * d],
            [2 * d, ddiag, d, 2 * d, 0],
        ])

        weights_back = np.array([
            [0, 2*d, d, ddiag, 2 * d],
            [2*d, 0, ddiag, d, 2*d],
            [d, ddiag, 0, d, d],
            [ddiag, d, d, 0,  d],
            [2 * d, 2*d, d,  d, 0],
        ])

        # Find connections
        [rows, cols] = np.where(L == 1)
        # For computational/memory reasons, initialize the velocity vector
        dxi = np.zeros((2, self.N))
        # Initialize leader state
        state = 0
        # Limit maximum linear speed of any robot
        magnitude_limit = 0.15
        # Create gains for our formation control algorithm
        formation_control_gain = 20
        desired_distance = 0.15
        si_to_uni_dyn = self.parameter.uni_dynamics()
        si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()
        # Single-integrator position controller
        leader_controller = create_si_position_controller(x_velocity_gain=0.5, y_velocity_gain=0.3, velocity_magnitude_limit=0.7)

        direction = 0

        for t in range(100000):
            if direction == 0:
                weights = weights_forward
                leader_idx = 0
                follower_idx = [1, 2, 3, 4]
            else:
                weights = weights_back
                leader_idx = 4
                follower_idx = [0, 1, 2, 3]
            x = self.nav.facility.get_poses()
            # Followers
            for i in follower_idx:
                # Zero velocities and get the topological neighbors of agent i
                dxi[:, [i]] = np.zeros((2, 1))
                neighbors = topological_neighbors(L, i)
                for j in neighbors:
                    dxi[:, [i]] += formation_control_gain * (
                                np.power(np.linalg.norm(x[:2, [j]] - x[:2, [i]]), 2) - np.power(weights[i, j],
                                                                                                2)) * (
                                               x[:2, [j]] - x[:2, [i]])
            # Leader
            waypoint = waypoints[:, state].reshape((2, 1))
            dxi[:, [leader_idx]] = leader_controller(x[:2, [leader_idx]], waypoint)
            if np.linalg.norm(x[:2, [leader_idx]] - waypoint) < close_enough:
                direction = 1 - direction
                state = (state + 1) % 2

            # Keep single integrator control vectors under specified magnitude
            # Threshold control inputs
            norms = np.linalg.norm(dxi, 2, 0)
            idxs_to_normalize = (norms > magnitude_limit)
            dxi[:, idxs_to_normalize] *= magnitude_limit / norms[idxs_to_normalize]
            # Use barriers and convert single-integrator to unicycle commands
            dxi = si_barrier_cert(dxi, x[:2, :])
            dxu = si_to_uni_dyn(dxi, x)
            print("direction:", direction)
            print("state:", state)
            print("distance:", np.linalg.norm(x[:2, [0]] - waypoint))
            print("wp:", waypoint)
            print("follower:", follower_idx)
            print("x", x[:, leader_idx])
            # Set the velocities of agents 1,...,N to dxu
            self.nav.facility.set_velocities(np.arange(self.N), dxu)
            # Iterate the simulation
            if self.debug == 1:
                self.nav.facility.step()
            else:
                self.nav.facility.step_real()
                time.sleep(0.033)
                self.nav.facility.get_real_position()


if __name__ == "__main__":
    fc = LeaderFollower()
    fc.leader_follower()
