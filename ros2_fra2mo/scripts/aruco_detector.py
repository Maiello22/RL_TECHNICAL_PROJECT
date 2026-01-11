#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Int32

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Subscriber ai marker rilevati
        self.aruco_sub = self.create_subscription(
            MarkerArray,
            '/aruco_marker_publisher/markers',
            self.aruco_callback,
            10
        )
        
        # Publisher per notificare i marker rilevati
        self.marker_detected_pub = self.create_publisher(
            Int32,
            '/aruco_detected',
            10
        )
        
        self.detected_markers = set()  # Evita duplicati
        
        self.get_logger().info('🔍 Fra2mo ArUco Detector started')
    
    def aruco_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            marker_id = marker.id
            
            # Se è un marker nuovo, notifica
            if marker_id not in self.detected_markers:
                self.detected_markers.add(marker_id)
                
                # Pubblica l'ID del marker rilevato
                marker_msg = Int32()
                marker_msg.data = marker_id
                self.marker_detected_pub.publish(marker_msg)
                
                self.get_logger().info(
                    f'✅ NEW ARUCO DETECTED: Marker {marker_id}\n'
                    f'   Notifying arm navigator...'
                )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()