from navigationInterface import Observation, ObservationAdapter, MotionSolutionAdapter, NavigationAdapter
from gradfInterface import CELLDictPtr, CellDictAdapter, CELLDict


def test1():
    workspace = CellDictAdapter()
    workspace.set_int("ndim", 2)
    workspace.set_list_double("range_min", [-2.0, -1.0])
    workspace.set_list_double("range_max", [2.0, 1.0])
    workspace.set_double("margin", 0.2)

    constraint = CellDictAdapter()
    constraint.set_string("type", "distance")
    constraint.set_double("value", 0.1)

    algorithm = CellDictAdapter()
    algorithm.set_string("method", "rvo")
    parameters = CellDictAdapter()
    parameters.set_double("timeStep", 0.1)
    parameters.set_double("neighborDistance", 0.5)
    parameters.set_double("maxNeighbors", 10)
    parameters.set_double("timeHorizon", 5.)
    parameters.set_double("timeHorizonObstacle", 5.)
    parameters.set_double("radius", 0.1)
    parameters.set_double("maxSpeed", 0.1)
    algorithm.set_cell("parameters", parameters.get_dict_pointer())
    # print("three")

    json = CellDictAdapter()
    json.set_cell("workspace", workspace.get_dict_pointer())
    json.set_int("num_agent", 2)
    json.set_list_double("start", [0.0, 0.0,
                                   0.0, 0.0])
    json.set_list_double("goal", [0.0, 0.0,
                                  0.0, 0.0])
    json.set_list_cell("constraints", [constraint.get_dict_pointer()])
    json.set_cell("algorithm", algorithm.get_dict_pointer())
    a = json.get_dict_pointer()
    navigator = NavigationAdapter(a)

    # navigator = NavigationAdapter("./rvo_obstacles_test.json", "motionPlanning")
    # solution = navigator.run()
    navigator.reset([-0.64, -0.35, 0.1, 0.2], [-0.4, 0.0, 0.3, 0.4])
    # navigator = NavigationAdapter("./zerostarts.json", "pathPlanning")
    # navigator.plan("parallel")

    observation_wrapper = ObservationAdapter()
    observation_wrapper.set_positions([-0.64, -0.35, 0.1, 0.2])
    observation_wrapper.add_obstacles([0.1, 0.1, 0.2, 0.1, 0.3, 0.2, 0.4, 0.5])
    observation_wrapper.set_emergencies([True, False])
    observe = observation_wrapper.get_value()
    # navigator.delete_all_constraints()
    # navigator.add_obstacle([0.5, 0.1, 0.6, 0.1, 0.6, 0.5, 0.5, 0.5])

    solution = MotionSolutionAdapter(navigator.step(observe))


def test2():
    workspace = CellDictAdapter()
    workspace.set_int("ndim", 2)
    workspace.set_list_double("range_min", [-2.0, -1.0])
    workspace.set_list_double("range_max", [2.0, 1.0])
    workspace.set_double("margin", 0.2)

    constraint = CellDictAdapter()
    constraint.set_string("type", "distance")
    constraint.set_double("value", 0.1)

    algorithm = CellDictAdapter()
    algorithm.set_string("method", "rvo")
    parameters = CellDictAdapter()
    parameters.set_double("timeStep", 0.1)
    parameters.set_double("neighborDistance", 0.5)
    parameters.set_double("maxNeighbors", 10)
    parameters.set_double("timeHorizon", 5.)
    parameters.set_double("timeHorizonObstacle", 5.)
    parameters.set_double("radius", 0.1)
    parameters.set_double("maxSpeed", 0.07)
    # a = parameters.get_dict_pointer()
    # print("test pybind type")
    algorithm.set_cell("parameters", parameters.get_dict_pointer())
    # print("three")

    json = CellDictAdapter()
    json.set_cell("workspace", workspace.get_dict_pointer())
    json.set_int("num_agent", 2)
    json.set_list_double("start", [0.1, 0.1,
                                   0.8, 0.1])
    json.set_list_double("goal", [0.8, 0.1,
                                  0.1, 0.1])
    json.set_list_cell("constraints", [constraint.get_dict_pointer()])
    json.set_cell("algorithm", algorithm.get_dict_pointer())
    navigator = NavigationAdapter(json.get_dict_pointer())

    observation_wrapper = ObservationAdapter()
    observation_wrapper.set_positions([-0.64, -0.35, 0.1, 0.2])
    observation_wrapper.add_obstacles([0.1, 0.1, 0.2, 0.1, 0.3, 0.2, 0.4, 0.5])
    observation_wrapper.set_emergencies([True, False])
    # navigator.step(observation_wrapper.get_value())
    solution = MotionSolutionAdapter(navigator.step(observation_wrapper.get_value()))


def printPath(path=[]):
    print("trajectory: ")
    robotSize = 8
    print("robot 0: ")
    for i in range(int(len(path)/robotSize)):
        print("     ({},{})".format(round(path[i * robotSize + 0], 2), round(path[i * robotSize + 1], 2)))
    print("robot 1: ")
    for i in range(int(len(path) / robotSize)):
        print("     ({},{})".format(round(path[i * robotSize + 2], 2), round(path[i * robotSize + 3], 2)))
    print("robot 2: ")
    for i in range(int(len(path) / robotSize)):
        print("     ({},{})".format(round(path[i * robotSize + 4], 2), round(path[i * robotSize + 5], 2)))
    print("robot 3: ")
    for i in range(int(len(path) / robotSize)):
        print("     ({},{})".format(round(path[i * robotSize + 6], 2), round(path[i * robotSize + 7], 2)))



def test3():
    navigator = NavigationAdapter("fourstarts.json")
    navigator.plan("whca")
    path = navigator.get_path()
    printPath(path)

    start = [0.85, -0.149,
             0.35, -0.75,
             1.15, -0.75,
             -0.4, 0.8]
    goal = [0.85, -0.149,
            0.35, -0.75,
            1.15, -0.75,
            -0.05, 0.45]
    navigator.reset(start, goal)

    path = navigator.get_path()
    printPath(path)

    observation = ObservationAdapter()
    


if __name__ == '__main__':
    # test1()
    # test2()
    test3()
    print("testing")
