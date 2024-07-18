import sys
import enum
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node, Parameter
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PointStamped, PolygonStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from shapely.geometry import Polygon, Point as sgPoint
from rclpy.node import ReentrantCallbackGroup
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Quaternion
import pickle

from . import topological_map
from .signpost import Signpost
from .poly_creator_parameters import poly_creator_parameters


class Button(enum.IntEnum):
    NEXT = 1
    CONTINUE = 2
    BREAK = 3
    STOP = 4

class Tools(enum.IntEnum):
    CREATE_POLYGON = 0
    CREATE_EDGE = 1
    EDIT_CENTER = 2
    EDIT_CARDINAL_POINTS = 3
    DELETE_POLYGON = 4

class PolyCreatorNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("poly_creator", parameter_overrides=parameter_overrides) #Init node
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("PolyCreator node initialized")
        
        self._is_creating_polygon = False
        self._is_creating_edge = False
        self._is_editing_center = False
        self._edge_start = None
        self._vertices = []
        self._topological_map = topological_map.RectilinearGridMap()
        
        # ROS2 parameters
        self.param_listener = poly_creator_parameters.ParamListener(self)

        self._selected_tool = Tools.CREATE_POLYGON
        self.get_logger().info("Selected tool: CREATE_POLYGON")

        # ROS2 interfaces
        self.callback_group = ReentrantCallbackGroup()
        self.click_listener = self.create_subscription(PointStamped, "clicked_point", self.click_callback, 10, callback_group=self.callback_group)
        self.rviz_visual_tools_subscriber = self.create_subscription(Joy, "/rviz_visual_tools_gui", self.handle_button_callback, 10, callback_group=self.callback_group)
        self.poly_publisher = self.create_publisher(MarkerArray, "poly", 10)
        self.create_timer(0.1, self.display_polygons_callback)
        
        if len(self.get_parameter("import_path").value) > 0:
            self.import_polygons()
    
    @property
    def selected_tool(self):
        return self._selected_tool
    
    @selected_tool.setter
    def selected_tool(self, tool: Tools):
        if tool == self._selected_tool:
            return
        self._selected_tool = tool
        self.get_logger().info(f"Selected tool: {tool.name}")
        
    def import_polygons(self):
        self.get_logger().info(f"Importing polygons from {self.get_parameter('import_path').value}")
        self._topological_map = topological_map.RectilinearGridMap.read_from_file(self.get_parameter("import_path").value)
        self.get_logger().info(f"Imported map:\n {self._topological_map}")
    
    def export_polygons(self):
        self.get_logger().info(f"Exporting polygons to {self.get_parameter('export_path').value}")
        self.get_logger().info(f"Exported map:\n {self._topological_map}")
        self._topological_map.write_to_file(self.get_parameter("export_path").value)
    
    def _create_polygon_marker(self, poly: Polygon, id: int):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.lifetime = Duration(seconds=0.2).to_msg()
        marker.points = [Point(x=x, y=y) for x, y in poly.exterior.coords]
        return marker

    def _get_vertices_marker(self):
        if len(self._vertices) == 0:
            return None
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0 #0 is reserved for the vertices marker
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.lifetime = Duration(seconds=0.2).to_msg()
        marker.points = [Point(x=x, y=y) for x, y in self._vertices]
        return marker
    
    def _create_edge_markers(self, node: topological_map.Node, edge_dict: dict[topological_map.Direction, Node], id: int):
        markers = []
        i = 1
        for direction in (topological_map.Direction.NORTH, topological_map.Direction.EAST):
            if not edge_dict[direction]:
                continue
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = id + i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.color.a = 1.0
            marker.color.b = 1.0
            marker.lifetime = Duration(seconds=0.2).to_msg()
            marker.points = [Point(x=node.cardinal_center[0], y=node.cardinal_center[1]), Point(x=edge_dict[direction].cardinal_center[0], y=edge_dict[direction].cardinal_center[1])]
            markers.append(marker)
            i+=1
        return markers
    
    def _cancel_current_operation(self):
        if self._is_creating_polygon:
            self.selected_tool = Tools.CREATE_POLYGON
            self._is_creating_polygon = False
            self.get_logger().info("Polygon creation cancelled")
            self._vertices = []
        if self._is_creating_edge:
            self.selected_tool = Tools.CREATE_EDGE
            self._edge_start = None
            self.get_logger().info("Edge creation cancelled")
    
    def display_polygons_callback(self):
        msg = MarkerArray()
        for id, node in enumerate(self._topological_map.get_edges().keys()):
            poly = node.area
            marker = self._create_polygon_marker(poly, id*3 + 1) #3 markers per polygon
            msg.markers.append(marker)
            markers = self._create_edge_markers(node, self._topological_map.get_edges()[node], id*3 + 1)
            msg.markers.extend(markers)
        marker = self._get_vertices_marker()
        if marker:
            msg.markers.append(marker)
        self.poly_publisher.publish(msg)
        if self.get_parameter("show_cardinal_points").value:
            msg = MarkerArray()
            for node in self._topological_map.edges:
                if sum(map(lambda x: 1 if x is not None else 0, self._topological_map.edges[node].values())) == 0:
                    continue
                relative_directions = self._topological_map.get_relative_directions(node)
                cardinal_points = {topological_map.Direction.NORTH: node.cardinal_center + relative_directions[topological_map.Direction.NORTH] * node.north_offset,
                                        topological_map.Direction.EAST: node.cardinal_center + relative_directions[topological_map.Direction.EAST] * node.east_offset,
                                        topological_map.Direction.SOUTH: node.cardinal_center + relative_directions[topological_map.Direction.SOUTH] * node.south_offset,
                                        topological_map.Direction.WEST: node.cardinal_center + relative_directions[topological_map.Direction.WEST] * node.west_offset}
                for direction, point in cardinal_points.items():
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.id = hash((node, direction, 0)) % 2147483647
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
                    marker.scale.z = 0.2
                    marker.color.a = 1.0
                    if direction == topological_map.Direction.NORTH:
                        rgb = (1.0, 0.0, 0.0)
                    elif direction == topological_map.Direction.EAST:
                        rgb = (0.0, 1.0, 0.0)
                    elif direction == topological_map.Direction.SOUTH:
                        rgb = (0.0, 0.0, 1.0)
                    elif direction == topological_map.Direction.WEST:
                        rgb = (1.0, 1.0, 0.0)
                    marker.color.r = rgb[0]
                    marker.color.g = rgb[1]
                    marker.color.b = rgb[2]
                    marker.lifetime = Duration(seconds=0.2).to_msg()
                    marker.pose.position.x = point[0]
                    marker.pose.position.y = point[1]
                    marker.pose.position.z = 0.1
                    msg.markers.append(marker)
            self.poly_publisher.publish(msg)
        if self.get_parameter("show_neighbors").value:
            msg = MarkerArray()
            for node in self._topological_map.edges:
                if sum(map(lambda x: 1 if x is not None else 0, self._topological_map.edges[node].values())) == 0:
                    continue
                relative_directions = self._topological_map.get_relative_directions(node)
                for direction, vect in relative_directions.items():
                    if self._topological_map.edges[node][direction] is None:
                        continue
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.id = hash((node, direction,25565)) % 2147483647 # Let's hope this is unique!
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    marker.scale.x = 1.0 # Length of the arrow
                    marker.scale.y = 0.2 # Width of the arrow
                    marker.scale.z = 0.2 # Height of the arrow
                    marker.color.a = 1.0 
                    if direction == topological_map.Direction.NORTH:
                        rgb = (1.0, 0.0, 0.0)
                    elif direction == topological_map.Direction.EAST:
                        rgb = (0.0, 1.0, 0.0)
                    elif direction == topological_map.Direction.SOUTH:
                        rgb = (0.0, 0.0, 1.0)
                    elif direction == topological_map.Direction.WEST:
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
                    msg.markers.append(marker)
            self.poly_publisher.publish(msg)
    
    def handle_button_callback(self, msg: Joy):
        if msg.buttons[Button.NEXT] == 1 and self._is_creating_polygon:
            if len(self._vertices) < 3:
                self.get_logger().error("A polygon must have at least 3 vertices")
                return
            poly = Polygon(self._vertices)
            self._topological_map.add_node(topological_map.Node(poly))
            self._is_creating_polygon = False
            self.get_logger().info(f"Polygon created: {poly}")
            self._vertices = []
            self.selected_tool = Tools.CREATE_POLYGON
        elif msg.buttons[Button.CONTINUE] == 1:
            self._cancel_current_operation()
            if self.selected_tool == Tools.CREATE_POLYGON:
                self.selected_tool = Tools.CREATE_EDGE
            elif self.selected_tool == Tools.CREATE_EDGE:
                self.selected_tool = Tools.EDIT_CENTER
            elif self.selected_tool == Tools.EDIT_CENTER:
                self.selected_tool = Tools.EDIT_CARDINAL_POINTS
            elif self.selected_tool == Tools.EDIT_CARDINAL_POINTS:
                self.selected_tool = Tools.DELETE_POLYGON
            elif self.selected_tool == Tools.DELETE_POLYGON:
                self.selected_tool = Tools.CREATE_POLYGON
        elif msg.buttons[Button.BREAK] == 1:
            self._cancel_current_operation()
        elif msg.buttons[Button.STOP] == 1:
            self._cancel_current_operation()
            self.export_polygons()
    
    def _get_direction(self):
        direction = input("Enter the direction of the edge (N, E, S, W): ")
        while direction not in ("N", "E", "S", "W"):
            direction = input("Invalid direction. Enter the direction of the edge (N, E, S, W): ")
        if direction == "N":
            direction = topological_map.Direction.NORTH
        elif direction == "E":
            direction = topological_map.Direction.EAST
        elif direction == "S":
            direction = topological_map.Direction.SOUTH
        else:
            direction = topological_map.Direction.WEST
        return direction
    
    def click_callback(self, msg: PointStamped):
        point = msg.point
        node = self._topological_map.localize(sgPoint(point.x, point.y))
        if node:
            if self._is_creating_polygon:
                self._cancel_current_operation()
            if self.selected_tool == Tools.CREATE_EDGE:
                if not self._is_creating_edge:
                    self.get_logger().info(f"Edge creation started from node {node.centroid}")
                    self._edge_start = node
                    self._is_creating_edge = True
                else:
                    if self._edge_start == node:
                        self.get_logger().error("An edge cannot connect a node to itself")
                        return
                    direction = self._get_direction()
                    self._topological_map.add_node(node, self._edge_start, direction)
                    self.get_logger().info(f"Edge created from {self._edge_start.centroid} to {node.centroid} with direction {direction.name}")
                    self._is_creating_edge = False
                    self._edge_start = None
            elif self.selected_tool == Tools.EDIT_CENTER:
                node.cardinal_center = np.asarray([point.x, point.y])
                self.get_logger().info(f"Node center edited to {point}")
            elif self.selected_tool == Tools.DELETE_POLYGON:
                self._topological_map.delete_node(node)
                self.get_logger().info(f"Node {node.centroid} deleted")
            elif self.selected_tool == Tools.EDIT_CARDINAL_POINTS:
                self.get_logger().info(f"Editing cardinal points of node {node.centroid}")
                self.get_logger().info(f"Current offsets: N: {node.north_offset}, E: {node.east_offset}, S: {node.south_offset}, W: {node.west_offset}")
                direction = self._get_direction()
                offset = float(input(f"Enter the new {direction.name} offset: "))
                if direction == topological_map.Direction.NORTH:
                    node.north_offset = offset
                elif direction == topological_map.Direction.EAST:
                    node.east_offset = offset
                elif direction == topological_map.Direction.SOUTH:
                    node.south_offset = offset
                else:
                    node.west_offset = offset
                self.get_logger().info(f"Cardinal point {direction.name} offset edited to {offset}")

            return
        if self._is_creating_edge:
            self._cancel_current_operation()
        if not self._is_creating_polygon:
            self._is_creating_polygon = True
            self.get_logger().info(f"Creating new polygon")
        self.get_logger().info(f"Appending point: {point.x}, {point.y}")
        self._vertices.append((point.x, point.y))

def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = PolyCreatorNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main(sys.argv)
