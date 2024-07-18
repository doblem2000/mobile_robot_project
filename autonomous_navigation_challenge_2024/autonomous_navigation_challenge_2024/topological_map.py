# This file contains the implementation of the topological map that will be used by the robot to navigate the environment.
import enum
import math
import numpy as np
import pickle
from rclpy.duration import Duration
import shapely.geometry as sg
from visualization_msgs.msg import MarkerArray, Marker

from geometry_msgs.msg import Pose, Quaternion
from .signpost import Signpost
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class Vect2D():
    @staticmethod
    def rotate_90_counter_clockwise(vect):
        return np.array([-vect[1], vect[0]])
    @staticmethod
    def rotate_90_clockwise(vect):
        return np.array([vect[1], -vect[0]])
    @staticmethod
    def rotate(vect, angle):
        return np.array([vect[0] * np.cos(angle) - vect[1] * np.sin(angle), vect[0] * np.sin(angle) + vect[1] * np.cos(angle)])

def normalize(vect):
    return vect / np.linalg.norm(vect)

class Direction(enum.IntEnum):
    """
        Directions are relative to the local topology of a node to adapt to real-world scenarios in which the grid is not uniformly oriented.
    """
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    def get_opposite(d):
        return (d + 2) % 4
    def get_adjacents(d):
        return [(d + 1) % 4, (d + 3) % 4]
    def turn_left(d):
        return (d + 3) % 4
    def turn_right(d):
        return (d + 1) % 4
    
class Node():
    def __init__(self, area: sg.Polygon):
        self.area = area # Polygon representing the area covered by the node in the map frame
        centroid = area.centroid # Point representing the center of the node in the map frame
        self.centroid = np.asarray([centroid.x, centroid.y]) # Coordinates of the center of the node in the map frame
        self.cardinal_center = self.centroid # Coordinates of the cardinal center of the node in the map frame
        self.relative_directions = None # Directions of the neighbors relative to the node
        self.cardinal_points = None # Cardinal points of the node in the map frame
        self.north_offset = 2.0 # Offset of the north cardinal point from the centroid
        self.east_offset = 3.0 # Offset of the east cardinal point from the centroid
        self.south_offset = 2.0 # Offset of the south cardinal point from the centroid
        self.west_offset = 3.0 # Offset of the west cardinal point from the centroid
    
    def get_centroid_pose(self, direction: Direction):
        return self.get_point_pose(self.centroid, direction)
    
    def get_center_pose(self, direction: Direction):
        return self.get_point_pose(self.cardinal_center, direction)
    
    def get_point_pose(self, point: np.ndarray, direction: Direction):
        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]
        rotation = np.arctan2(self.relative_directions[direction][1], self.relative_directions[direction][0])
        pose.orientation.z = math.sin(rotation / 2)
        pose.orientation.w = math.cos(rotation / 2)
        return pose
    
    def get_closest_direction(self, observer_forward):
        node_relative_directions_distances = [(d,np.dot(observer_forward, v)) for d,v in self.relative_directions.items()]
        observer_direction: Direction = max(node_relative_directions_distances, key=lambda x: x[1])[0]
        return observer_direction
            
class RectilinearGridMap():
    """
        A topological map represented as a rectilinear grid of nodes.
        The gird is statically defined and does not change during the execution of the program.
        The origin of the grid is at the bottom-left corner of the map.
        Each node has a maximum of 4 neighbors, one for each direction (NORTH, EAST, SOUTH, WEST).
    """
    def __init__(self):
        self.origin: Node = None
        self.edges: dict[Node, dict[Direction, Node]] = {}
        self.built = False
        
    @staticmethod
    def read_from_file(file_path: str):
        with open(file_path, "rb") as f:
            import_dict = pickle.load(f)
            map = import_dict["map"]
            map.built = False
            map.relative_directions = None
            map.cardinal_points = None
            for node in map.edges:
                if not hasattr(node, "north_offset"):
                    node.north_offset = 2.0
                    node.east_offset = 3.0
                    node.south_offset = 2.0
                    node.west_offset = 3.0
                if not hasattr(node, "cardinal_center"):
                    node.cardinal_center = node.centroid
        return map
    
    def write_to_file(self, file_path: str):
        self.built = False
        self.cardinal_points = None
        self.relative_directions = None
        export_dict = {"map": self}
        with open(file_path, "wb") as f:
            pickle.dump(export_dict, f, pickle.HIGHEST_PROTOCOL)
    
    def add_node(self, node: Node, parent: Node = None, direction: Direction = None):
        """Add a node to the map and connect it to its parent node in the specified direction."""
        if self.built:
            raise Exception("The map has already been built.")
        if not self.origin:
            self.origin = node
            self.edges = {self.origin: {Direction.NORTH: None, Direction.EAST: None, Direction.SOUTH: None, Direction.WEST: None}}
            return True
        if parent is None:
            self.edges[node] = {Direction.NORTH: None, Direction.EAST: None, Direction.SOUTH: None, Direction.WEST: None}
            return True
        if direction is None or parent not in self.edges:
            return False
        self.edges[parent][direction] = node
        self.edges[node][Direction.get_opposite(direction)] = parent
        return True
    
    def __str__(self):
        result = ""
        result += f"Origin: {self.origin.centroid if self.origin else None}\n"
        for node in self.edges:
            result += f"Node {node} {node.centroid}:\n"
            for direction, neighbor in self.edges[node].items():
                result += f"\t{direction.name}: {neighbor.centroid if neighbor else None}\n"
        return result
    
    def prune(self):
        """Remove nodes that are not connected to the origin node."""
        if self.built:
            raise Exception("The map has already been built.")
        visited = set()
        queue = [self.origin]
        while queue:
            node = queue.pop(0)
            visited.add(node)
            for neighbor in self.edges[node].values():
                if neighbor and neighbor not in visited:
                    queue.append(neighbor)
        for node in self.edges:
            if node not in visited:
                self.delete_node(node)
    
    def delete_node(self, node: Node):
        """Remove a node from the map and update the connections between its neighbors."""
        if self.built:
            raise Exception("The map has already been built.")
        if node not in self.edges:
            return False
        if self.origin == node:
            self.origin = None
            self.edges = {}
            return True
        for direction, neighbor in self.edges[node].items():
            if neighbor:
                self.edges[neighbor][Direction.get_opposite(direction)] = None
        del self.edges[node]
        return True
    
    def get_edges(self):
        return self.edges
    
    def localize(self, point: sg.Point):
        """Given a point on the map, find the node that contains it."""
        for node in self.edges:
            if node.area.contains(point):
                return node
        return None
    
    def get_neighbor(self, node: Node, direction: Direction):
        """Given a node and a direction, return the neighbor node in that direction."""
        return self.edges[node][direction]
    
    def get_neighbors(self, node: Node) -> dict[Direction, Node]:
        return self.edges[node]
    
    def get_relative_directions(self, node: Node):
        neighbors = self.get_neighbors(node)
        #print(f"DEBUG: {neighbors}")
        neighbors_vect = {k: normalize(v.cardinal_center - node.cardinal_center) if v else None for k,v in neighbors.items()}
        
        # If a neighbor is None, set the direction to the opposite of the direction of the opposite neighbor
        for direction, vect in neighbors_vect.items():
            if vect is None:
                opposite_direction = Direction.get_opposite(direction)
                if not neighbors_vect[opposite_direction] is None:
                    neighbors_vect[direction] = -neighbors_vect[opposite_direction]
        
        # If a neighbor is still None, set the direction to the average of the directions of the adjacent neighbors
        if neighbors_vect[Direction.NORTH] is None:
            n1 = Vect2D.rotate_90_clockwise(neighbors_vect[Direction.WEST])
            n2 = Vect2D.rotate_90_counter_clockwise(neighbors_vect[Direction.EAST])
            s1 = Vect2D.rotate_90_counter_clockwise(neighbors_vect[Direction.WEST])
            s2 = Vect2D.rotate_90_clockwise(neighbors_vect[Direction.EAST])
            neighbors_vect[Direction.NORTH] = normalize(n1 + n2)
            neighbors_vect[Direction.SOUTH] = normalize(s1 + s2)
        if neighbors_vect[Direction.EAST] is None:
            e1 = Vect2D.rotate_90_clockwise(neighbors_vect[Direction.NORTH])
            e2 = Vect2D.rotate_90_counter_clockwise(neighbors_vect[Direction.SOUTH])
            w1 = Vect2D.rotate_90_counter_clockwise(neighbors_vect[Direction.NORTH])
            w2 = Vect2D.rotate_90_clockwise(neighbors_vect[Direction.SOUTH])
            neighbors_vect[Direction.EAST] = normalize(e1 + e2)
            neighbors_vect[Direction.WEST] = normalize(w1 + w2)
        
        return neighbors_vect
        
    def build(self):
        if self.built:
            return
        self.prune()
        for node in self.edges:
            node.relative_directions = self.get_relative_directions(node)
            node.cardinal_points = {Direction.NORTH: node.cardinal_center + node.relative_directions[Direction.NORTH] * node.north_offset,
                                    Direction.EAST: node.cardinal_center + node.relative_directions[Direction.EAST] * node.east_offset,
                                    Direction.SOUTH: node.cardinal_center + node.relative_directions[Direction.SOUTH] * node.south_offset,
                                    Direction.WEST: node.cardinal_center + node.relative_directions[Direction.WEST] * node.west_offset}
        self.built = True
    
    def interpret_signpost_by_pose(self, observer_pose: Pose, signpost: Signpost):
        if not self.built:
            raise Exception("The map has not been built yet.")
        # Localize the observer in the map
        observer_node = self.localize(sg.Point(observer_pose.position.x, observer_pose.position.y))
        if observer_node is None:
            return None, None
        observer_angle = euler_from_quaternion([observer_pose.orientation.x, observer_pose.orientation.y, observer_pose.orientation.z, observer_pose.orientation.w])[2]
        observer_forward = np.array([np.cos(observer_angle), np.sin(observer_angle)])
        observer_direction: Direction = observer_node.get_closest_direction(observer_forward)
        return self.interpret_signpost_by_direction(observer_node, observer_direction, signpost)
    
    def interpret_signpost_by_direction(self, observer_node: Node, observer_direction: Direction, signpost: Signpost):
        if not self.built:
            raise Exception("The map has not been built yet.")
        print(f"Observer direction: {Direction(observer_direction).name}")
        if signpost == Signpost.LEFT:
            next_node_direction = Direction.turn_left(observer_direction)
        elif signpost == Signpost.RIGHT:
            next_node_direction = Direction.turn_right(observer_direction)
        elif signpost == Signpost.FORWARD:
            next_node_direction = observer_direction
        elif signpost == Signpost.BACKWARD:
            next_node_direction = Direction.get_opposite(observer_direction)
        elif signpost == Signpost.STOP:
            return (observer_node, observer_direction)
        else:
            return None, None
        return (self.get_neighbor(observer_node, next_node_direction), next_node_direction)
    
    def get_all_relative_directions_msg(self, timestamp):
        if not self.built:
            raise Exception("The map has not been built yet.")
        msg = MarkerArray()
        for node in self.edges:
            msg.markers.extend(self._get_relative_direction_msgs(node, timestamp))
        return msg
    
    def get_all_neighbors_msg(self, timestamp):
        if not self.built:
            raise Exception("The map has not been built yet.")
        msg = MarkerArray()
        for node in self.edges:
            msg.markers.extend(self._get_neighbors_msgs(node, timestamp))
        return msg
    
    def get_all_cardinal_points_msg(self, timestamp):
        if not self.built:
            raise Exception("The map has not been built yet.")
        msg = MarkerArray()
        for node in self.edges:
            msg.markers.extend(self._get_cardinal_point_msgs(node, timestamp))
        return msg

    def _get_cardinal_point_msgs(self, node: Node, timestamp):
        res = []
        for direction, point in node.cardinal_points.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = timestamp
            marker.id = hash((node, direction, 0)) % 2147483647
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            if direction == Direction.NORTH:
                rgb = (1.0, 0.0, 0.0)
            elif direction == Direction.EAST:
                rgb = (0.0, 1.0, 0.0)
            elif direction == Direction.SOUTH:
                rgb = (0.0, 0.0, 1.0)
            elif direction == Direction.WEST:
                rgb = (1.0, 1.0, 0.0)
            marker.color.r = rgb[0]
            marker.color.g = rgb[1]
            marker.color.b = rgb[2]
            marker.lifetime = Duration(seconds=0.2).to_msg()
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.1
            res.append(marker)
        return res
                

    def _get_relative_direction_msgs(self, node: Node, timestamp):
        res = []
        relative_directions = node.relative_directions
        for direction, vect in relative_directions.items():
            if vect is not None:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = timestamp
                marker.id = hash((node, direction, 1)) % 2147483647 # Let's hope this is unique!
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.scale.x = 1.0 # Length of the arrow
                marker.scale.y = 0.2 # Width of the arrow
                marker.scale.z = 0.2 # Height of the arrow
                marker.color.a = 1.0 
                if direction == Direction.NORTH:
                    rgb = (1.0, 0.0, 0.0)
                elif direction == Direction.EAST:
                    rgb = (0.0, 1.0, 0.0)
                elif direction == Direction.SOUTH:
                    rgb = (0.0, 0.0, 1.0)
                elif direction == Direction.WEST:
                    rgb = (1.0, 1.0, 0.0)
                marker.color.r = rgb[0]
                marker.color.g = rgb[1]
                marker.color.b = rgb[2]
                marker.lifetime = Duration(seconds=0.2).to_msg()
                marker.pose.position.x = node.cardinal_center[0]
                marker.pose.position.y = node.cardinal_center[1]
                marker.pose.position.z = 0.1
                q = quaternion_from_euler(0, 0, np.arctan2(vect[1], vect[0]))
                marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                res.append(marker)
        return res
    
    def _get_neighbors_msgs(self, node: Node, timestamp):
        res = []
        relative_directions = node.relative_directions
        for direction, vect in relative_directions.items():
            if self.edges[node][direction] is None:
                continue
            if vect is not None:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = timestamp
                marker.id = hash((node, direction, 2)) % 2147483647 # Let's hope this is unique!
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.scale.x = 1.0 # Length of the arrow
                marker.scale.y = 0.2 # Width of the arrow
                marker.scale.z = 0.2 # Height of the arrow
                marker.color.a = 1.0 
                if direction == Direction.NORTH:
                    rgb = (1.0, 0.0, 0.0)
                elif direction == Direction.EAST:
                    rgb = (0.0, 1.0, 0.0)
                elif direction == Direction.SOUTH:
                    rgb = (0.0, 0.0, 1.0)
                elif direction == Direction.WEST:
                    rgb = (1.0, 1.0, 0.0)
                marker.color.r = rgb[0]
                marker.color.g = rgb[1]
                marker.color.b = rgb[2]
                marker.lifetime = Duration(seconds=0.2).to_msg()
                marker.pose.position.x = node.cardinal_center[0]
                marker.pose.position.y = node.cardinal_center[1]
                marker.pose.position.z = 0.1
                q = quaternion_from_euler(0, 0, np.arctan2(vect[1], vect[0]))
                marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                res.append(marker)
        return res