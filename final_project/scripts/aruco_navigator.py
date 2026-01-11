#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float64MultiArray
import math
import time

class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')
        
        # Flag per iniziare navigazione solo dopo ArUco ID 10
        self.start_navigation = False
        
        # ✅ STATO PULSANTE
        self.button_position = None
        self.button_pressed = False
        self.button_open_position = 0.0075   # Posizione aperto
        self.button_closed_position = 0.00025   # Posizione chiuso (premuto)
        self.button_press_threshold = 0.00025   # Soglia per considerare premuto
        
        # Subscriber per ArUco
        self.aruco_detected_sub = self.create_subscription(Int32, '/aruco_detected', self.aruco_detected_callback, 10)
        # Subscriber pulsante
        self.button_sub = self.create_subscription(JointState, '/aruco_cube_button/joint_states', self.button_callback, 10)
        # Publisher cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/arm/cmd_vel', 10)
        
        # Subscriber Lidar e Odometry
        self.scan_sub = self.create_subscription(LaserScan, '/arm/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/arm/odom', self.odom_callback, 10)
        
        # Publisher braccio e gripper
        self.arm_joint_pub = self.create_publisher(Float64MultiArray, '/arm/position_controller/commands', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/arm/gripper_controller/commands', 10)
        
        # Posizione spawn
        self.spawn_map_x = -9.0
        self.spawn_map_y = 0.0
        
        # Posizione odometria
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.current_yaw = 0.0
        
        # Velocità
        self.linear_speed = 0.5
        self.angular_speed = 0.6
        
        # Ostacoli
        self.min_obstacle_distance = 0.6
        self.obstacle_detected = False
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # Stati navigazione
        self.navigating = False
        self.target_map_x = None
        self.target_map_y = None
        self.direction_toggle = 0
        self.following_clear_path = False
        self.clear_path_timer = 0
        
        self.get_logger().info('🤖 ArUco Navigator initialized')
        self.get_logger().info('⏳ Waiting for ArUco ID 10 to start navigation...')
        self.get_logger().info(f'📍 Spawn position (map frame): ({self.spawn_map_x}, {self.spawn_map_y})')
    
    # ==================== CALLBACKS ====================
    def aruco_detected_callback(self, msg):
        if msg.data == 10 and not self.start_navigation:
            self.start_navigation = True
            self.get_logger().info(
                f'\n🎉 ArUco ID 10 DETECTED! Starting navigation...\n'
            )
    
    def button_callback(self, msg: JointState):
        """✅ Callback per lo stato del pulsante"""
        if 'button_joint' in msg.name:
            idx = msg.name.index('button_joint')
            self.button_position = msg.position[idx]
            
            # Verifica se premuto (posizione negativa < soglia)
            was_pressed = self.button_pressed
            self.button_pressed = self.button_position < self.button_press_threshold
            
            # Log cambio stato
            if self.button_pressed and not was_pressed:
                self.get_logger().info(
                    f'🔴 BUTTON PRESSED! Position: {self.button_position:.4f}'
                )
            elif not self.button_pressed and was_pressed:
                self.get_logger().info(
                    f'⚪ Button released. Position: {self.button_position:.4f}'
                )
    
    def odom_to_map(self, odom_x, odom_y):
        return self.spawn_map_x + odom_x, self.spawn_map_y + odom_y
    
    def scan_callback(self, msg):
        front_ranges, left_ranges, right_ranges = [], [], []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        for i, r in enumerate(msg.ranges):
            angle_deg = math.degrees(angle_min + i * angle_increment)
            if -20 <= angle_deg <= 20:
                front_ranges.append(r)
            elif 20 < angle_deg <= 70:
                left_ranges.append(r)
            elif -70 <= angle_deg < -20:
                right_ranges.append(r)
        
        def get_min_valid(ranges):
            valid = [r for r in ranges if msg.range_min < r < msg.range_max]
            return min(valid) if valid else float('inf')
        
        self.front_distance = get_min_valid(front_ranges)
        self.left_distance = get_min_valid(left_ranges)
        self.right_distance = get_min_valid(right_ranges)
        
        # Larghezza ostacolo frontale
        self.obstacle_width_deg = 0.0
        if front_ranges:
            obstructed = [i for i, r in enumerate(front_ranges) if r < 1.0]
            if obstructed:
                min_idx, max_idx = min(obstructed), max(obstructed)
                angle_min_obst = angle_min + min_idx * angle_increment
                angle_max_obst = angle_min + max_idx * angle_increment
                self.obstacle_width_deg = math.degrees(angle_max_obst - angle_min_obst)
        
        self.obstacle_detected = self.front_distance < self.min_obstacle_distance
        
        current_time = time.time()
        if not hasattr(self, 'last_log_time') or current_time - self.last_log_time > 2.0:
            self.get_logger().info(
                f'📡 Lidar distances - Front: {self.front_distance:.2f}m, '
                f'Left: {self.left_distance:.2f}m, Right: {self.right_distance:.2f}m | '
                f'Obstacle width: {self.obstacle_width_deg:.1f}°'
            )
            self.last_log_time = current_time
    
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        siny = 2 * (o.w * o.z + o.x * o.y)
        cosy = 1 - 2 * (o.y**2 + o.z**2)
        self.current_yaw = math.atan2(siny, cosy)
    
    # ==================== MOVIMENTO ROBOT ====================
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def rotate_to_angle(self, target_yaw_rad):
        def normalize(angle):
            while angle > math.pi: angle -= 2*math.pi
            while angle < -math.pi: angle += 2*math.pi
            return angle
        target_yaw_rad = normalize(target_yaw_rad)
        twist = Twist()
        while rclpy.ok():
            diff = normalize(target_yaw_rad - self.current_yaw)
            if abs(diff) < 0.05: break
            twist.angular.z = max(-self.angular_speed, min(self.angular_speed, 2.0*diff))
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.stop_robot()
    
    def avoid_obstacle(self):
        self.get_logger().warn(f'⚠️ Obstacle detected! Front: {self.front_distance:.2f}m')
        self.stop_robot()
        time.sleep(0.2)
        # Retromarcia
        twist = Twist()
        twist.linear.x = -0.2
        for _ in range(20):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.stop_robot()
        time.sleep(0.2)
        # Decidi dove girare
        base_angle = max(60, self.obstacle_width_deg + 30)
        if self.left_distance > self.right_distance + 0.2:
            turn_angle, direction = math.radians(base_angle), "LEFT"
        elif self.right_distance > self.left_distance + 0.2:
            turn_angle, direction = math.radians(-base_angle), "RIGHT"
        else:
            turn_angle = math.radians(base_angle if self.direction_toggle %2==0 else -base_angle)
            direction = "LEFT" if self.direction_toggle %2==0 else "RIGHT"
            self.direction_toggle += 1
        self.get_logger().info(f'🔄 Turning {direction} ({math.degrees(abs(turn_angle)):.0f}°)')
        self.rotate_to_angle(self.current_yaw + turn_angle)
        self.following_clear_path = True
        self.clear_path_timer = 100
    
    def navigate_to_map_position(self, map_x, map_y, tolerance=0.15):
        self.target_map_x = map_x
        self.target_map_y = map_y
        self.navigating = True
        rate = self.create_rate(10)
        stuck_counter = 0
        max_stuck = 50
        while rclpy.ok() and self.navigating:
            cur_x, cur_y = self.odom_to_map(self.odom_x, self.odom_y)
            dx, dy = map_x - cur_x, map_y - cur_y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < tolerance:
                self.stop_robot()
                self.navigating = False
                return True
            # Segui direzione libera dopo evitamento
            if self.following_clear_path:
                self.clear_path_timer -= 1
                if self.clear_path_timer <= 0:
                    self.following_clear_path = False
                else:
                    twist = Twist()
                    twist.linear.x = min(0.4, self.linear_speed+0.1)
                    self.cmd_vel_pub.publish(twist)
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue
            angle_diff = math.atan2(dy, dx) - self.current_yaw
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi
            if self.obstacle_detected:
                stuck_counter += 1
                if stuck_counter > max_stuck:
                    self.get_logger().error('❌ Stuck! Cannot reach target')
                    self.stop_robot()
                    return False
                self.avoid_obstacle()
                continue
            else:
                stuck_counter = 0
            twist = Twist()
            if abs(angle_diff) > 0.2:
                twist.angular.z = max(-self.angular_speed, min(self.angular_speed, 2.0*angle_diff))
                twist.linear.x = 0.0
            else:
                twist.linear.x = min(self.linear_speed, dist*0.5)
                twist.angular.z = 1.0*angle_diff
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.stop_robot()
        return False
    
    # ==================== COMANDO GIUNTI ====================
    def move_arm(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.arm_joint_pub.publish(msg)
        self.get_logger().info(f'🤖 Moving arm joints to {positions}')
    
    def move_gripper(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'🤖 Moving gripper joints to {positions}')
    
    # ✅ NUOVA FUNZIONE: Verifica pulsante premuto (ATTESA INFINITA)
    def wait_for_button_press(self, check_interval=0.1):
        """
        Attende INDEFINITAMENTE che il pulsante venga premuto.
        Non c'è timeout - aspetta finché non viene premuto.
        
        Returns:
            bool: True quando il pulsante è stato premuto
        """
        self.get_logger().info(
            '⏳ Waiting for button press (no timeout - waiting indefinitely)...'
        )
        
        start_time = time.time()
        
        while rclpy.ok():
            # Pulsante premuto!
            if self.button_pressed:
                elapsed = time.time() - start_time
                self.get_logger().info(
                    f'✅ Button pressed detected after {elapsed:.1f}s! '
                    f'Position: {self.button_position:.4f}'
                )
                return True
            
            # Log periodico ogni 5 secondi
            elapsed = time.time() - start_time
            if elapsed > 0 and int(elapsed) % 5 == 0:
                self.get_logger().info(
                    f'⏳ Still waiting... ({int(elapsed)}s elapsed)',
                    throttle_duration_sec=5.0
                )
            
            # Spin per ricevere messaggi
            rclpy.spin_once(self, timeout_sec=check_interval)
        
        return False
    
    # ✅ NUOVA FUNZIONE: Sequenza completa press button (1 SOLO TENTATIVO)
    def press_button_sequence(self):
        """
        Esegue la sequenza per premere il pulsante UNA SOLA VOLTA.
        Aspetta indefinitamente che il pulsante venga premuto.
        
        Returns:
            bool: True quando il pulsante è stato premuto
        """
        self.get_logger().info(
            f'\n{"="*60}\n'
            f'   🎯 Pressing button (single attempt)...\n'
            f'{"="*60}\n'
        )
        
        # Reset flag
        self.button_pressed = False
        
        # 1. Muovi braccio verso il pulsante
        self.get_logger().info('📍 Step 1/3: Moving arm to button position')
        self.move_arm([-0.7, -0.6, -0.6, 0.0])
        
        start = time.time()
        while rclpy.ok() and time.time() - start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # 2. Chiudi gripper (premi il pulsante)
        self.get_logger().info('📍 Step 2/3: Closing gripper to press button')
        self.move_gripper([self.button_closed_position, self.button_closed_position])
        
        # Attendi un momento per permettere la pressione
        start = time.time()
        while rclpy.ok() and time.time() - start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # 3. Aspetta INDEFINITAMENTE che il pulsante venga premuto
        self.get_logger().info('📍 Step 3/3: Waiting for button press (indefinitely)')
        
        # Questa funzione aspetta finché il pulsante non è premuto
        if self.wait_for_button_press():
            self.get_logger().info(
                f'\n{"="*60}\n'
                f'   ✅ SUCCESS! Button pressed!\n'
                f'{"="*60}\n'
            )
            
            # Apri gripper dopo successo
            self.get_logger().info('🔓 Opening gripper')
            self.move_gripper([0.03,0.03])
            time.sleep(1.0)
            
            return True
        
        # Se arriviamo qui, è stato interrotto (Ctrl+C)
        return False


# ==================== MAIN ====================
def main():
    rclpy.init()
    navigator = ArucoNavigator()
    
    time.sleep(2)
    navigator.get_logger().info('⏳ Waiting for ArUco ID 10...')
    while rclpy.ok() and not navigator.start_navigation:
        rclpy.spin_once(navigator, timeout_sec=0.1)
    
    if not navigator.start_navigation:
        navigator.get_logger().error('❌ Navigation cancelled')
        navigator.destroy_node()
        rclpy.shutdown()
        return
    
    # Target: cubo ArUco con pulsante
    target_x, target_y = 1.0, -2.3
    navigator.get_logger().info(
        f'\n🚀 Starting navigation to ArUco cube at ({target_x}, {target_y})\n'
    )
    
    # Naviga verso il cubo
    success = navigator.navigate_to_map_position(target_x, target_y)
    
    if success:
        navigator.get_logger().info('✅ Navigation completed successfully')
        
        # ✅ ESEGUI SEQUENZA PRESSIONE PULSANTE (1 TENTATIVO, ATTESA INFINITA)
        button_success = navigator.press_button_sequence()
        
        if button_success:
            navigator.get_logger().info(
                f'\n🎉 Mission accomplished! Button pressed successfully!\n'
            )
        else:
            navigator.get_logger().error(
                f'\n❌ Mission failed! Could not press button\n'
            )
    else:
        navigator.get_logger().error('❌ Navigation failed - could not reach target')
    
    time.sleep(0.5)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()