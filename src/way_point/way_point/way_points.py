import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseStamped



class WayPointsServer(Node):
    def __init__(self) :
        super().__init__("way_points_server")
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        # self.create_service()

        self.declare_parameter("pose_position_x_list", [0.,0.,0.,0.,0.,0.])
        self.declare_parameter("pose_position_y_list", [0.,0.,0.,0.,0.,0.])
        self.declare_parameter("pose_position_z_list", [0.,0.,0.,0.,0.,0.])

        self.declare_parameter("pose_orientation_x_list", [0.,0.,0.,0.,0.,0.])
        self.declare_parameter("pose_orientation_y_list", [0.,0.,0.,0.,0.,0.])
        self.declare_parameter("pose_orientation_z_list", [0.,0.,0.,0.,0.,0.])
        self.declare_parameter("pose_orientation_w_list", [0.,0.,0.,0.,0.,0.])

        (self.pose_position_x_list, 
        self.pose_position_y_list, 
        self.pose_position_z_list, 
        self.pose_orientation_x_list, 
        self.pose_orientation_y_list, 
        self.pose_orientation_z_list, 
        self.pose_orientation_w_list) = self.get_parameters(["pose_position_x_list", "pose_position_y_list", "pose_position_z_list",
                                                             "pose_orientation_x_list", "pose_orientation_y_list", "pose_orientation_z_list", "pose_orientation_w_list"])
        
        
        self.get_logger().info("test_msg")
        self.get_logger().info(str(self.pose_orientation_w_list.value))

        self.move()

    def move(self):

        for index, target in enumerate(self.pose_position_x_list.value):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.pose_position_x_list.value[index]
            goal_pose.pose.position.y = self.pose_position_y_list.value[index]
            goal_pose.pose.position.z = self.pose_position_z_list.value[index]
            goal_pose.pose.orientation.x = self.pose_orientation_x_list.value[index]
            goal_pose.pose.orientation.y = self.pose_orientation_y_list.value[index]
            goal_pose.pose.orientation.z = self.pose_orientation_z_list.value[index]
            goal_pose.pose.orientation.w = self.pose_orientation_w_list.value[index]
            self.nav.goToPose(goal_pose)
            i = 0
            while not self.nav.isTaskComplete():
                i = i + 1
                feedback = self.nav.getFeedback()
                if feedback and i % 5 == 0 :
                    print("distance remaining: " + "{:.2f}".format(feedback.distance_remaining) + " meters.")
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=40.0):
                        self.nav.cancelTask()
                    # if feedback.distance_remaining < 0.25:
                    #     break
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print("goal succeeded!!")
            elif result == TaskResult.CANCELED:
                print("goal canceled")
                break
            elif result == TaskResult.FAILED:
                print("gaol failed")
                break
                


def main(args = None) : 
    rclpy.init(args=args)

    server = WayPointsServer()
    rclpy.spin(server) # while 1 :
    
    server.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__" : 
    main()