import sys
from angle import *
from problem_spec import ProblemSpec
from robot_config import *
from math import *
import random
from tester import *
"""
Template file for you to implement your solution to Assignment 2. Contains a class you can use to represent graph nodes,
and a method for finding a path in a graph made up of GraphNode objects.

COMP3702 2020 Assignment 2 Support Code
"""


class GraphNode:
    """
    Class representing a node in the state graph. You should create an instance of this class each time you generate
    a sample.
    """
    
    def __init__(self, spec, config):
        """
        Create a new graph node object for the given config.

        Neighbors should be added by appending to self.neighbors after creating each new GraphNode.

        :param spec: ProblemSpec object
        :param config: the RobotConfig object to be stored in this node
        """
        self.spec = spec
        self.config = config
        self.neighbors = []

    def __eq__(self, other):
        return test_config_equality(self.config, other.config, self.spec)

    def __hash__(self):
        return hash(tuple(self.config.points))

    def get_successors(self):
        return self.neighbors

    @staticmethod
    def add_connection(n1, n2):
        """
        Creates a neighbor connection between the 2 given GraphNode objects.

        :param n1: a GraphNode object
        :param n2: a GraphNode object
        """
        n1.neighbors.append(n2)
        n2.neighbors.append(n1)


def find_graph_path(spec, init_node):
    """
    This method performs a breadth first search of the state graph and return a list of configs which form a path
    through the state graph between the initial and the goal. Note that this path will not satisfy the primitive step
    requirement - you will need to interpolate between the configs in the returned list.

    You may use this method in your solver if you wish, or can implement your own graph search algorithm to improve
    performance.

    :param spec: ProblemSpec object
    :param init_node: GraphNode object for the initial configuration
    :return: List of configs forming a path through the graph from initial to goal
    """
    # search the graph
    init_container = [init_node]

    # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        if test_config_equality(current.config, spec.goal, spec):
            # found path to goal
            return init_visited[current]

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                init_container.append(suc)
                init_visited[suc] = init_visited[current] + [suc.config]
    return None
def interpolation (config1,config2,spec, rate = 1):
    # config1_angles = []
    # config2_angles = []
    ee1x = spec.grapple_points[0][0]
    ee1y = spec.grapple_points[0][1]
    #inter_angles = [config1.ee1_angles,config2.ee2_angles]
    mid_angle = []
    inter_configs = [config1,config2] 
    i = 0
    j = 1
    for i in range(spec.num_segments):
        mid_angle.append(Angle(radians = (config1.ee1_angles[i].in_radians() + config2.ee1_angles[i].in_radians())/2))
        i += 1
    # inter_angles = inter_angles[:1] + [mid_angle] + inter_angles[1:]
    config = make_robot_config_from_ee1(ee1x,ee1y,mid_angle,config1.lengths,ee1_grappled=True)
    inter_configs = inter_configs[:1] + [config] + inter_configs[1:]
    # print(config_distance_config(inter_configs[0],inter_configs[1],spec))
   
    while config_distance_config(inter_configs[-2],inter_configs[-1],spec) > spec.PRIMITIVE_STEP * rate:
        # node_distance = config_distance_config(inter_configs[-2],inter_configs[-1],spec)
        inter_configs2 = interpolation(inter_configs[j-1],inter_configs[j],spec, rate = rate)
        inter_configs = inter_configs[:j] + inter_configs2 + inter_configs[j:]
        # print('113113')
        # node_distance = config_distance(inter_configs[j-1],inter_configs[j],spec)
        j = j + (len(inter_configs2)) + 1
        # print('j=',j)
    return inter_configs[1:len(inter_configs)-1]

def sample (spec,segements,angles = [],lengths = [],ee1x = None,
            ee1y = None,ee2x = None,ee2y = None,ee1_grappled = True):
    segements = spec.num_segments
    ee1x = spec.grapple_points[0][0]
    ee1y = spec.grapple_points[0][1]
    # lengths = spec.max_lengths
    # ee2x = 
    # ee2y = 
    i = 0
    num = 50
    configurations = []
    j = 1
    while i < num:
        for j in range(segements):
            a = random.uniform(-165.0,165.0)
            b = Angle(degrees = a)
            angles.append(b)
            l = random.uniform(spec.min_lengths[j],spec.max_lengths[j])
            lengths.append(l) 
            j += 1
        config = make_robot_config_from_ee1 (ee1x, ee1y, angles, lengths, ee1_grappled=True)
        angles = []
        lengths = []
        self_collision = test_self_collision(config, spec)
        obstacle_collision = test_obstacle_collision(config,spec,spec.obstacles)  
        if self_collision == True:
            if obstacle_collision == True:
                configurations.append(GraphNode(spec,config))
                i += 1
            
    print('loop_end')
    return configurations

def config_distance_config(c1, c2, spec):
    max_ee1_delta = 0
    max_ee2_delta = 0
    for i in range(spec.num_segments):
        if abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians()) > max_ee1_delta:
            max_ee1_delta = abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians())
        if abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians()) > max_ee2_delta:
            max_ee2_delta = abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians())

    # measure leniently - allow compliance from EE1 or EE2
    max_delta = min(max_ee1_delta, max_ee2_delta)

    for i in range(spec.num_segments):
        if abs(c2.lengths[i] - c1.lengths[i]) > max_delta:
            max_delta = abs(c2.lengths[i] - c1.lengths[i])
    return max_delta

def config_distance_graphnode(c1, c2, spec):
    max_ee1_delta = 0
    max_ee2_delta = 0
    for i in range(spec.num_segments):
        if abs((c2.config.ee1_angles[i] - c1.config.ee1_angles[i]).in_radians()) > max_ee1_delta:
            max_ee1_delta = abs((c2.config.ee1_angles[i] - c1.config.ee1_angles[i]).in_radians())
        if abs((c2.config.ee2_angles[i] - c1.config.ee2_angles[i]).in_radians()) > max_ee2_delta:
            max_ee2_delta = abs((c2.config.ee2_angles[i] - c1.config.ee2_angles[i]).in_radians())

    # measure leniently - allow compliance from EE1 or EE2
    max_delta = min(max_ee1_delta, max_ee2_delta)

    for i in range(spec.num_segments):
        if abs(c2.config.lengths[i] - c1.config.lengths[i]) > max_delta:
            max_delta = abs(c2.config.lengths[i] - c1.config.lengths[i])
    return max_delta


def main(arglist):
    #input_file = arglist[0]
    #output_file = arglist[1]
    #input_file = r'C:\Users\fyc19\Desktop\ai\assignment-2-support-code-master\assignment-2-support-code-master\testcases\3g1_m0.txt'
    # 'C:/Users/fyc19/Desktop/ai/assignment-2-support-code-master/assignment-2-support-code-master/testcase/3g1_m0.txt'
    # r'C:\Users\fyc19\Desktop\ai\assignment-2-support-code-master\assignment-2-support-code-master\testcase\3g1_m0.txt'
    input_file = './testcases/3g1_m0.txt'
    output_file = 'asdasdas'
    spec = ProblemSpec(input_file)
    n = 0
    i = 0
    j = 0
    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)
    sampling = sample(spec,segements = spec.num_segments)
    sampling = [init_node] + sampling + [goal_node]
    for i in range(0,len(sampling)):   
        for j in range(i+1, len(sampling)): 
            node_distance = config_distance_graphnode(sampling[i],sampling[j],spec)
            # print(node_distance)
            if node_distance > spec.PRIMITIVE_STEP:
                interpolation_check = interpolation(sampling[i].config,sampling[j].config,spec, rate = 20)
                for n in range(len(interpolation_check)):
                    obstacle_collision = test_obstacle_collision(interpolation_check[n],spec,spec.obstacles)
                    if obstacle_collision == False:
                        break
                if obstacle_collision:
                    sampling[i].add_connection(sampling[i],sampling[j])
            j += 1
        i += 1
    path = find_graph_path(spec,init_node)
    steps = []
    if path == None:
        print ('None')
        return
    steps.append(path[0])
    for i in range(1, len(path)):
        steps += interpolation(path[i-1],path[i],spec)
        steps += [path[i]]

    #
    #
    # Code for your main method can go here.
    #
    # Your code should find a sequence of RobotConfig objects such that all configurations are collision free, the
    # distance between 2 successive configurations is less than 1 primitive step, the first configuration is the initial
    # state and the last configuration is the goal state.
    #
    #

    write_robot_config_list_to_file(output_file, steps)

    # You may uncomment this line to launch visualiser once a solution has been found. This may be useful for debugging.
    # *** Make sure this line is commented out when you submit to Gradescope ***
    #


if __name__ == '__main__':
    main(sys.argv[1:])
