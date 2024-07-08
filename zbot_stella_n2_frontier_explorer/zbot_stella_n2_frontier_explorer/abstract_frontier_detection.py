from abc import ABC, abstractmethod
from geometry_msgs.msg import Pose

class FrontierDetection(ABC):
    def __init__(self, params=None):
        self.map_data = None
        self.current_pose = None
        self.params = params or {}

    def update_map(self, map_data):
        """
        Update the map data.
        
        :param map_data: The SLAM map data.
        """
        self.map_data = map_data

    def update_pose(self, pose: Pose):
        """
        Update the current pose of the robot.
        
        :param pose: The current pose of the robot.
        """
        self.current_pose = pose
    
    @abstractmethod
    def get_frontiers(self):
        """
        Get frontiers in the map.
        
        :return: A list of frontiers.
        """
        raise NotImplementedError