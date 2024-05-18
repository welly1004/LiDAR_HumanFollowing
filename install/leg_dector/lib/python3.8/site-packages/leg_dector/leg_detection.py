import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
from math import sqrt

class LegDetectionNode(Node):
    def __init__(self):
        super().__init__('leg_detection')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.marker_publisher = self.create_publisher(Marker, '/leg_marker', 10)
        self.arrow_publisher = self.create_publisher(Marker, '/vector_arrows', 10)
        self.front_arrow_publisher = self.create_publisher(Marker, '/front_arrow', 10)
        self.distance_publisher = self.create_publisher(Marker, '/vector_distance', 10)
        
        self.shutdown_distance_threshold = 0.2  # 關閉節點的距離閾值，單位為米

    def scan_callback(self, msg):
        leg_points = self.detect_leg_points(msg)
        if leg_points is not None and len(leg_points)>0:
            # leg_points=[leg_points[0]]    #
            markers = self.publish_leg_markers(msg, leg_points)
            self.publish_vector_arrows(msg.header.frame_id, leg_points)
            self.publish_vector_distance(msg.header.frame_id, leg_points)
            for marker in markers:
                self.marker_publisher.publish(marker)
        else:
            self.clear_markers(msg.header.frame_id)
            self.clear_arrows(msg.header.frame_id)
            self.clear_distance(msg.header.frame_id)
        self.publish_front_arrow(msg.header.frame_id)

        if hasattr(self, 'target_detected') and self.target_detected:
            distance_to_target = self.calculate_distance([0, 0], self.target_position)
            if distance_to_target < self.shutdown_distance_threshold:
                self.get_logger().info('Distance to target is less than 20cm. Shutting down node.')
                rclpy.shutdown()  # 關閉ROS節點
                return

    def detect_leg_points(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
       
        if not hasattr(self, 'target_detected'):
            self.target_detected = False
        
        if not self.target_detected:
        
            valid_mask = (ranges > 0.1) & (ranges < 0.35)
        else:
            
            valid_mask = (ranges < 3.0)
            

        ranges = ranges[valid_mask]
        angles = angles[valid_mask]
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        points = np.column_stack((xs, ys))

        clustering_successful = False
        if len(points) > 0:
            clustering = DBSCAN(eps=0.1, min_samples=3).fit(points)
            clustering_successful = True

        leg_points = []
        
        
        if self.target_detected and clustering_successful:
            # target_index = self.find_nearest_point_index(points, self.target_position)
            valid_mask = np.linalg.norm(points - self.target_position,axis=1) < 0.2
            if np.sum(valid_mask)!=0:
                points = points[valid_mask]
                # clustering = DBSCAN(eps=0.1, min_samples=3).fit(points)  
                cluster_mean = np.mean(points, axis=0)
                self.target_position = cluster_mean
            
            leg_points.append(self.target_position)
            # if np.any(ranges < 0.20):
            #     rclpy.shutdown()  # 如果存在距离小于30厘米的点，则关闭ROS节点
            #     return []  # 返回空列表，停止检测

        if clustering_successful and (not self.target_detected):   
        # else:
            for label in set(clustering.labels_):
                if label != -1:
                    cluster = points[clustering.labels_ == label]
                    pca = PCA(n_components=1)
                    pca.fit(cluster)
                    variance_ratio = pca.explained_variance_ratio_[0]
                    if variance_ratio < 0.95:
                        cluster_mean = np.mean(cluster, axis=0)
                        leg_points.append(cluster_mean)
                        if not self.target_detected:
                            self.target_detected = True
                            self.target_position = cluster_mean
            
        return leg_points if leg_points else []




    # def find_nearest_point_index(self,points, target_position):
    #     distances = np.linalg.norm(points - target_position, axis=1)
    #     return np.argmin(distances)

        
    def publish_vector_arrows(self, frame_id, leg_points):
        for i, center_of_cluster in enumerate(leg_points):
            arrow_marker = Marker()
            arrow_marker.header.frame_id = frame_id
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.points.append(Point(x=0.0, y=0.0, z=0.0))
            arrow_marker.points.append(Point(x=center_of_cluster[0], y=center_of_cluster[1], z=0.0))
            arrow_marker.scale.x = 0.02
            arrow_marker.scale.y = 0.04
            arrow_marker.scale.z = 0.0
            arrow_marker.color.a = 1.0
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 1.0
            arrow_marker.color.b = 0.0
            # Calculate distance and set as text
            self.arrow_publisher.publish(arrow_marker)


    def publish_vector_distance(self, frame_id, leg_points):
        for i, center_of_cluster in enumerate(leg_points):
            arrow_text_marker = Marker()
            arrow_text_marker.header.frame_id = frame_id
            arrow_text_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_text_marker.type =  Marker.TEXT_VIEW_FACING
            arrow_text_marker.action = Marker.ADD
            arrow_text_marker.pose.position.x = center_of_cluster[0] + 0.1
            arrow_text_marker.pose.position.y = center_of_cluster[1] + 0.1
            arrow_text_marker.pose.position.z = 0.0
            arrow_text_marker.pose.orientation.w = 1.0
            arrow_text_marker.scale.x = 0.02
            arrow_text_marker.scale.y = 0.05
            arrow_text_marker.scale.z = 0.06
            arrow_text_marker.color.a = 1.0
            arrow_text_marker.color.r = 0.0
            arrow_text_marker.color.g = 1.0
            arrow_text_marker.color.b = 0.0
            distance = self.calculate_distance([0, 0], center_of_cluster)
            arrow_text_marker.text = '{:.2f}m'.format(distance)
            self.distance_publisher.publish(arrow_text_marker)
    
    def calculate_distance(self, point1, point2):
        dx = point1[0] - point2[0]
        dy = point1[1] - point2[1]
        return sqrt(dx ** 2 + dy ** 2)

    def publish_leg_markers(self, scan_msg, leg_points):
        markers = []
        for i, cluster in enumerate(leg_points):
            marker_msg = Marker()
            marker_msg.header.frame_id = scan_msg.header.frame_id
            marker_msg.type = Marker.SPHERE_LIST
            marker_msg.action = Marker.ADD
            marker_msg.scale.x = 0.05
            marker_msg.scale.y = 0.05
            marker_msg.scale.z = 0.05
            marker_msg.color.a = 1.0
            marker_msg.color.r = 1.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            marker_msg.pose.orientation.w = 1.0
            marker_msg.points = [self.point_to_point(cluster)]
            markers.append(marker_msg)
        return markers
    
    def publish_front_arrow(self, frame_id):
        arrow_marker = Marker()
        arrow_marker.header.frame_id = frame_id
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        arrow_marker.points.append(Point(x=0.0, y=-0.3, z=0.0))
        arrow_marker.scale.x = 0.03
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05
        arrow_marker.color.a = 1.0
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 1.0
        self.front_arrow_publisher.publish(arrow_marker)

    def point_to_point(self, point):
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = 0.0
        return p
    
    def clear_markers(self,frame_id):
        delete_msg = Marker()
        delete_msg.header.frame_id = frame_id
        delete_msg.action = Marker.DELETEALL
        self.marker_publisher.publish(delete_msg)
    
    def clear_arrows(self,frame_id):
        arrow_marker = Marker()
        arrow_marker.header.frame_id = frame_id
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.DELETE
        self.arrow_publisher.publish(arrow_marker)

    def clear_distance(self,frame_id):
        arrow_marker = Marker()
        arrow_marker.header.frame_id = frame_id
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.DELETE
        self.distance_publisher.publish(arrow_marker)
        
   

def main(args=None):
    rclpy.init(args=args)
    leg_detection_node = LegDetectionNode()
    rclpy.spin(leg_detection_node)
    leg_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
