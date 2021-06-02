#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>


void save_pose(turtlesim::Pose &loc, const turtlesim::PoseConstPtr &pose) {
    loc = *pose;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "turtle_controller");

    ros::NodeHandle nh;

    struct Env {
        turtlesim::Pose self, other;
    } env;

    auto self_pose_sub = nh.subscribe<turtlesim::Pose>(
            nh.param<std::string>("name", "mpc_turtle") + "/pose", 1,
            [&loc = env.self](const turtlesim::PoseConstPtr &pose) {
                return save_pose(loc, pose);
            }
    );

    auto other_pose_sub = nh.subscribe<turtlesim::Pose>(
            nh.param<std::string>("other_name", "turtle1") + "/pose", 1,
            [&loc = env.other](const turtlesim::PoseConstPtr &pose) {
                return save_pose(loc, pose);
            }
    );

    auto vel_pub = nh.advertise<geometry_msgs::Twist>(nh.param<std::string>("name", "mpc_turtle") + "/cmd_vel", 1);
    geometry_msgs::Twist vel;

    ros::Duration(1.0).sleep(); // Lazy wait for first message
    ros::spinOnce();

    while (ros::ok()) {
        ROS_INFO("Running");
        ROS_INFO("(%f, %f), %f ", env.self.x, env.self.y, env.other.x);

        vel.linear.x = 0.7 * (env.other.x - env.self.x - 1);
        vel_pub.publish(vel);

        ros::Duration(1).sleep();
        ros::spinOnce();
    }
}