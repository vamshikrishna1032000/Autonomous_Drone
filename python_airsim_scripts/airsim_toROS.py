# Import AirSim and ROS packages
import airsim
import rospy
from airsim_ros_pkgs.msg import VelCmd, GimbalAngleEulerCmd

# Function to initialize ROS node and AirSim client
def initialize_ros_airsim():
    # Initialize the ROS node
    rospy.init_node('airsim_node', anonymous=True)
    
    # Connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    # Set up ROS publishers to send commands to AirSim
    vel_cmd_publisher = rospy.Publisher('/airsim_node/vel_cmd_body_frame', VelCmd, queue_size=10)
    gimbal_cmd_publisher = rospy.Publisher('/airsim_node/gimbal_angle_euler_cmd', GimbalAngleEulerCmd, queue_size=10)

    return client, vel_cmd_publisher, gimbal_cmd_publisher

# Function to send a velocity command via ROS
def send_velocity_command(vel_cmd_publisher, velocity):
    msg = VelCmd()
    msg.x_val = velocity[0]
    msg.y_val = velocity[1]
    msg.z_val = velocity[2]
    vel_cmd_publisher.publish(msg)