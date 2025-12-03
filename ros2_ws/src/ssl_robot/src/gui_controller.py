#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QGridLayout
from PyQt5.QtCore import QTimer, Qt

class RobotController(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.initUI()
        
        # Movement State
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # Keys pressed
        self.keys = {
            Qt.Key_W: False,
            Qt.Key_S: False,
            Qt.Key_A: False,
            Qt.Key_D: False,
            Qt.Key_Q: False,
            Qt.Key_E: False
        }

        # Timer to update ROS loop
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ros)
        self.timer.start(50) # 20Hz

    def initUI(self):
        self.setWindowTitle('Robocup SSL Controller')
        self.setGeometry(100, 100, 300, 200)
        
        layout = QVBoxLayout()
        
        self.label = QLabel("Use WASD to Move, QE to Rotate")
        self.label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.label)
        
        self.status_label = QLabel("Stopped")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.setLayout(layout)
        self.setFocusPolicy(Qt.StrongFocus) 

    def keyPressEvent(self, event):
        if event.key() in self.keys:
            self.keys[event.key()] = True

    def keyReleaseEvent(self, event):
        if event.key() in self.keys:
            self.keys[event.key()] = False

    def update_ros(self):
        speed = 1.0
        turn_speed = 2.0
        
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        if self.keys[Qt.Key_W]: self.linear_x += speed
        if self.keys[Qt.Key_S]: self.linear_x -= speed
        if self.keys[Qt.Key_A]: self.linear_y += speed  # Left in ROS body frame is +Y
        if self.keys[Qt.Key_D]: self.linear_y -= speed  # Right in ROS body frame is -Y
        if self.keys[Qt.Key_Q]: self.angular_z += turn_speed
        if self.keys[Qt.Key_E]: self.angular_z -= turn_speed
        
        # Publish Twist
        self.ros_node.publish_cmd(self.linear_x, self.linear_y, self.angular_z)
        
        # Update GUI
        status = f"X: {self.linear_x:.1f}, Y: {self.linear_y:.1f}, Z: {self.angular_z:.1f}"
        self.status_label.setText(status)

class ControllerNode(Node):
    def __init__(self):
        super().__init__('gui_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish_cmd(self, x, y, z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.angular.z = float(z)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ros_node = ControllerNode()
    
    app = QApplication(sys.argv)
    gui = RobotController(ros_node)
    gui.show()
    
    sys.exit(app.exec_())
    
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
