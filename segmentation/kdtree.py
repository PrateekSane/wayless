# Kdtree.py
import numpy as np
from sklearn.neighbors import KDTree

class KdTree_class:
    def __init__(self):
        self.tree = None
        self.points = None

    def insert_points(self, df, display_output=False):
        """
        Build KDTree from dataframe with columns X, Y, Z
        """
        self.points = df[["X", "Y", "Z"]].to_numpy()
        self.tree = KDTree(self.points)
        if display_output:
            print(f"KDTree built with {self.points.shape[0]} points")
        return self.tree  # keep API similar to your code

    def search_elements(self, node, search_point, distance_threshold, depth=0):
        """
        Query KDTree for neighbors within distance_threshold
        :param node: unused (for compatibility with your existing calls)
        :param search_point: (x, y, z) tuple
        :param distance_threshold: radius
        :param depth: unused (for compatibility)
        :return: indices of nearby points
        """
        if self.tree is None:
            return []

        search_point = np.array(search_point).reshape(1, -1)
        indices = self.tree.query_radius(search_point, r=distance_threshold)[0]
        return indices.tolist()
