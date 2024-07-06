import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import matplotlib.pyplot as plt
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray

from geometry_msgs.msg import Pose

OCC_THRESHOLD = 10 # Threshold for occupied cells
MIN_FRONTIER_SIZE = 5 

class OccupancyGrid2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx
    
class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8

class FrontierPoint():
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y

class FrontierCache():
    """
    Cache for frontier points.
    Using Cantor pairing function to hash x, y coordinates.
    """
    def __init__(self):
        self.cache = {}

    def get_point(self, x, y):
        idx = self.__cantor_hash(x, y)

        if idx in self.cache:
            return self.cache[idx]
        
        self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def __cantor_hash(self, x, y):
        # Cantor pairing function
        return (((x + y) * (x + y + 1)) / 2) + y

    def clear(self):
        self.cache = {}

class WavefrontFrontierExplorer(Node):
    def __init__(self):
        super().__init__('wavefront_frontier_explorer')
        self.get_logger().info('WavefrontFrontierExplorer initialized')


        self.frontier_marker_pub = self.create_publisher(
            MarkerArray, '/frontier_markers', QoSProfile(depth=1)
        )

        # Subscribe to the /map topic
        self.slam_map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.slam_map_callback, 
            QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        self.

        # self.navigator = BasicNavigator('basic_navigator')

        self.slam_map = None
        self.frontiers = []
        self.fCache = FrontierCache() # Cache for frontier points
        
        self.current_pose = Pose # Current robot pose

    def slam_map_callback(self, msg):
        self.slam_map = OccupancyGrid2d(msg)

    def __get_neighbours(self, point):
        # Get neighbors of a point
        neighbors = []

        for x in range(point.mapX - 1, point.mapX + 2):
            for y in range(point.mapY - 1, point.mapY + 2):
                if (x > 0 and x < self.slam_map.getSizeX() and y > 0 and y < self.slam_map.getSizeY()):
                    neighbors.append(self.fCache.getPoint(x, y))

        return neighbors
    
    def __find_free_point(self, mx, my):

        bfs_queue = [self.fCache.getPoint(mx, my)] # Breadth-first search queue

        while len(bfs_queue) > 0:
            loc = bfs_queue.pop(0)
            if self.slam_map.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
                return (loc.mapX, loc.mapY)
            
            for n in self.__get_neighbours(loc):
                if n.classification & PointClassification.MapClosed.value == 0:
                    n.classification = n.classification | PointClassification.MapClosed.value
                    bfs_queue.append(n)
        
        return (mx, my)
    
    def is_frontier_point(self, point):
        """
        Check if a point is a frontier point.
        A frontier point is a point that is adjacent to an unknown cell.
        """
        if self.slam_map.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
            return False
        
        has_free = False
        for n in self.__get_neighbours(point):
            cost = self.slam_map.getCost(n.mapX, n.mapY)

            if cost > OCC_THRESHOLD:
                return False
            
            if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
                has_free = True

        return has_free
    
    def __get_centroid(self, arr):
        arr = np.array(arr)
        length = arr.shape[0]
        sum_x = np.sum(arr[:, 0])
        sum_y = np.sum(arr[:, 1])
        return sum_x/length, sum_y/length

    def __get_frontier(self):
        
        self.fCache.clear()
        frontiers = []

        mx, my = self.slam_map.worldToMap(
            self.current_pose.position.x, self.current_pose.position.y
        ) # get robot position in map coordinates

        free_point = self.__find_free_point(mx, my)
        start_point = self.fCache.get_point(free_point[0], free_point[1])
        start_point.classification = PointClassification.MapOpen.value
        map_point_queue = [start_point] # Queue for outer breadth-first search

        while len(map_point_queue) > 0:
            p = map_point_queue.pop(0) 

            if p.classification & PointClassification.MapClosed.value != 0: # Skip if point is already closed
                continue

            if self.is_frontier_point(p):
                p.classification = p.classification | PointClassification.FrontierOpen.value
                frontier_queue = [p] # Queue for inner breadth-first search
                new_frontier = []

                while len(frontier_queue) > 0:
                    fpoint = frontier_queue.pop(0)

                    if fpoint.classification & (PointClassification.MapClosed.value | PointClassification.FrontierClosed.value) != 0:
                        continue

                    if self.is_frontier_point(fpoint):
                        new_frontier.append(fpoint)

                        for w in self.__get_neighbours(fpoint):
                            if w.classification & (PointClassification.FrontierOpen.value | 
                                                   PointClassification.FrontierClosed.value | 
                                                   PointClassification.MapClosed.value) == 0:
                                w.classification = w.classification | PointClassification.FrontierOpen.value
                                frontier_queue.append(w)

                    fpoint.classification = fpoint.classification | PointClassification.FrontierClosed.value
                
                new_frontier_cords = []
                for x in new_frontier:
                    x.classification = x.classification | PointClassification.MapClosed.value
                    new_frontier_cords.append(self.slam_map.mapToWorld(x.mapX, x.mapY))
                
                if len(new_frontier) > MIN_FRONTIER_SIZE: 
                    frontiers.append(self.__get_centroid(new_frontier_cords))

            # outer bfs
            for v in self.__get_neighbours(p):
                if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                    if any(self.slam_map.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in self.__get_neighbours(v)):
                        v.classification = v.classification | PointClassification.MapOpen.value
                        map_point_queue.append(v)

            p.classification = p.classification | PointClassification.MapClosed.value
    
        return frontiers



        

    
    



    

def main():
    rclpy.init()
    node = WavefrontFrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
