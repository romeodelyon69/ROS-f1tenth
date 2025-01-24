"""
ESE 680
RRT assignment
Author: Hongrui Zheng

This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math
import random

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
#from tf import transform_listener

realCar = False
infinity = float('inf')


if(realCar):
    lidarscan_topic = '/scan'
    loc_topic = '/pf/pose/odom'
    drive_topic = "/vesc/ackermann_cmd_mux/input/navigation"
else:
    lidarscan_topic = '/scan'
    loc_topic = '/odom'
    drive_topic = '/nav'

# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self):
        self.x = None
        self.y = None

        self.x_grid = None
        self.y_grid = None

        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False



# class def for RRT
class RRT(object):
    def __init__(self):
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.
        
        #map_topic = rospy.get_param('map')

        # TODO: create subscribers

        print("Don't foerget to launch : rosrun map_server map_server/romeoNZT/mymap.yaml")
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.marker_pub = rospy.Publisher ("/dynamic_viz",Marker, queue_size=100)

        # class attributes
        # TODO: maybe create your occupancy grid here
        self.L = 0.3
        self.occupancy_grid = OccupancyGrid()

        
        

    def map_callback(self, map_msg):
        self.occupancy_grid = map_msg
        print("map caracteristics : width", self.occupancy_grid.info.width, " height : ", self.occupancy_grid.info.height, " resolution : ", self.occupancy_grid.info.resolution)

        goal_x, goal_y = self.find_goal()

        
        tree = []

        origin_node = Node()
        origin_node.x = 0
        origin_node.y = 0

        origin_node.x_grid = int((0 - self.occupancy_grid.info.origin.position.x)/self.occupancy_grid.info.resolution)
        origin_node.y_grid = int((0 - self.occupancy_grid.info.origin.position.y)/self.occupancy_grid.info.resolution)

        origin_node.is_root = True

        tree.append(origin_node)
        print("origin_node : ", origin_node.x, origin_node.y, origin_node.x_grid, origin_node.y_grid)
        

        while(not self.is_goal(tree[-1], goal_x, goal_y)):
            printMarker(0,0,(0,1,1), 12345678, self.marker_pub)
            printMarker(goal_x,goal_y,(0,1,1), 123456789, self.marker_pub)
            print(len(tree))
            sample_point = self.sample()
            nearest_node = self.nearest(tree, sample_point)
            new_node = self.steer(nearest_node, sample_point)
            if(self.check_collision(nearest_node, new_node)):
                tree.append(new_node)
                printMarker(new_node.x, new_node.y, (1,0,0), len(tree), self.marker_pub)
        
        path = self.find_path(tree, tree[-1])

        self.print_path(path)


    def find_goal(self):
        return 2,0
    def sample(self):
        
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        while(True):
            x,y = random.randint(0, self.occupancy_grid.info.width-1), random.randint(0, self.occupancy_grid.info.height-1)
            print("random point : ", x,y)
            index = y * self.occupancy_grid.info.width + x
            if(self.occupancy_grid.data[index] < 20):
                x,y = x * self.occupancy_grid.info.resolution, y * self.occupancy_grid.info.resolution 
                x += self.occupancy_grid.info.origin.position.x
                y += self.occupancy_grid.info.origin.position.y
                return (x,y)
        



    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = None
        min_d = infinity

        for node in tree:
            d = dist((node.x, node.y), sampled_point)
            if d < min_d:
                min_d = d
                nearest_node = node
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        d = dist((nearest_node.x, nearest_node.y), sampled_point)

        vx = sampled_point[0] - nearest_node.x
        vy = sampled_point[1] - nearest_node.y

        #print(((nearest_node.x, nearest_node.y), sampled_point))

        x = nearest_node.x + vx/d*self.L
        y = nearest_node.y + vy/d*self.L

        new_node = Node()

        new_node.x = x
        new_node.y = y

        new_node.x_grid = int((x - self.occupancy_grid.info.origin.position.x)/self.occupancy_grid.info.resolution)
        new_node.y_grid = int((y - self.occupancy_grid.info.origin.position.y)/self.occupancy_grid.info.resolution)
        new_node.parent = nearest_node
        
        return new_node

    #on utilise l'algo de Bresenham parce qu'on fait des lignes droites dans une grille 
    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        x1,y1,x2,y2 = nearest_node.x_grid, nearest_node.y_grid, new_node.x_grid, new_node.y_grid
        dy = y2 -y1
        dx = x2- x1
        y = y1
        e = 0
        if(dx == 0):
            e10 = 4096
        else:
            e10 = dy/dx
        e01 = -1.0
        
        for x in range(x1, x2+1):
            index = y * self.occupancy_grid.info.width + x
            
            if(self.occupancy_grid.data[index] > 60):
                return False
            e = e + e10
            if(e > 0.5):
                y += 1
                e += e01

        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        d = dist((latest_added_node.x, latest_added_node.y), (goal_x, goal_y))
                 
        if d < self.L:
            print("salamalecoum mes petits loucoums")
            return True
        print("distance à la cible : ", d)
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        current_node = latest_added_node

        while(not current_node.is_root):
            path.append(current_node)
            current_node = current_node.parent
        path.reverse()
        return path
    
    def print_path(self, path):
        for i,node in enumerate(path):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = node.x
            marker.pose.position.y = node.y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            # Example
            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 0.6
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.marker_pub.publish(marker)




    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood
    
def dist(point1, point2):
    x1,y1,x2,y2 = point1[0], point1[1], point2[0], point2[1]
    return ((x1 - x2) **2 + (y1 - y2)**2)**(1/2)

def printMarker(x,y, colour, id, pub):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.id = id
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    # Example
    marker.scale.x = 0.6
    marker.scale.y = 0.6
    marker.scale.z = 0.6
    marker.color.a = 1.0
    marker.color.r = colour[0]
    marker.color.g = colour[1]
    marker.color.b = colour[2]
    pub.publish(marker)



def main():
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.spin()

if __name__ == '__main__':
    main()