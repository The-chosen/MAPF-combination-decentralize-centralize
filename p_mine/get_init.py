from gradfInterface import CellDictAdapter, CELLDictPtr
from utilities.controllers import *


class pios_facility_parameter:
    def __init__(self):
        self.method = "rvo"
        self.N = 1
        self.debug = 0
        self.linear_velocity_gain = 0.5
        self.angular_velocity_gain = 0.3
        self.velocity_magnitude_limit = 0.15
        self.angular_velocity_limit = 0.8
        self.position_error = 0.02
        self.position_epsilon = 0.01
        self.rotation_error = 0.1

    def get(self):
        return self.method, self.N, self.debug, self.position_error, self.rotation_error

    def rvo_init(self, obstacle_list, n):
        param = CellDictAdapter()
        param.set_double("robotCloseDist", 0.1)
        param.set_double("pointsDistance", 0.3)

        target = CellDictAdapter()
        target.set_list_double("goalConfiguration", self.N*[0.1, 0.1])
        configurationspace = CellDictAdapter()
        agv_0 = CellDictAdapter()
        agv_0.set_int("index", 0)
        agv_0.set_int("dimension", 2)
        agv_0.set_double("radius", 0.05)
        agv_0.set_int("wsIndex", 0)
        agv_0.set_list_double("position", [0.9, 0.9])
        agv_0.set_list_double("orientation", [0.0])
        agv_0.set_list_double("waypoint", [0.0, 0.0])
        agv_0.set_list_double("velocity", [0.0, 0.0])

        if obstacle_list:
            obstacle = CellDictAdapter()
            obstacle.set_string("type", "ObstaclePolygon")
            obstacle.set_list_double("value", obstacle_list[0])
        workspace = CellDictAdapter()
        workspace.set_int("index", 0)
        workspace.set_int("ndim", 2)
        workspace.set_list_double("range_min", [-0.3, -0.1])
        workspace.set_list_double("range_max", [2.3, 1.3])
        if obstacle_list:
            workspace.set_list_cell("obstacles", [obstacle.get_dict_pointer()])

        constraint = CellDictAdapter()
        constraint.set_int("dimension", 2*self.N)
        constraint_parameters = CellDictAdapter()
        constraint_parameters.set_string("type", "distance")
        constraint_parameters.set_double("value", 0.1)
        constraint.set_list_cell("parameters", [constraint_parameters.get_dict_pointer()])

        configurationspace.set_list_cell("agentState", self.N*[agv_0.get_dict_pointer()])
        configurationspace.set_list_cell("workspace", [workspace.get_dict_pointer()])
        configurationspace.set_cell("constraints", constraint.get_dict_pointer())

        algorithm = CellDictAdapter()
        algorithm.set_string("method", "rvo")
        parameters = CellDictAdapter()
        parameters.set_double("goalThreshold", 0.01)
        parameters.set_double("timeStep", 0.033)
        parameters.set_double("neighborDistance", 0.5)
        parameters.set_double("maxNeighbors", 10)
        parameters.set_double("timeHorizon", 5.)
        parameters.set_double("timeHorizonObstacle", 5.)
        parameters.set_double("radius", 0.08)
        parameters.set_double("maxSpeed", 0.1)
        algorithm.set_cell("parameters", parameters.get_dict_pointer())

        json_date = CellDictAdapter()
        json_date.set_cell("parameters", param.get_dict_pointer())
        json_date.set_cell("target", target.get_dict_pointer())
        json_date.set_cell("configurationspace", configurationspace.get_dict_pointer())
        json_date.set_cell("motionplanning", algorithm.get_dict_pointer())

        return json_date

    def controller(self):
        pose_controller = create_hybrid_unicycle_pose_controller(
            linear_velocity_gain=self.linear_velocity_gain,
            angular_velocity_gain=self.angular_velocity_gain,
            velocity_magnitude_limit=self.velocity_magnitude_limit,
            angular_velocity_limit=self.angular_velocity_limit,
            position_error=self.position_error,
            position_epsilon=self.position_epsilon,
            rotation_error=self.rotation_error
        )
        return pose_controller

    def uni_dynamics(self):
        dyn = create_si_to_uni_dynamics(linear_velocity_gain=self.linear_velocity_gain,
                                        angular_velocity_limit=self.angular_velocity_limit)
        return dyn