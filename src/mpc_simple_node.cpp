#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <mpc_simple/MPC.h>


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "turtle_controller");

    ros::NodeHandle nh;

    struct Env {
        turtlesim::Pose self, other;
    } env;

    // yuck
    auto self_pose_sub = nh.subscribe<turtlesim::Pose>(
            nh.param<std::string>("name", "mpc_turtle") + "/pose", 1,
            [&loc = env.self](const turtlesim::PoseConstPtr &pose) {
                loc = *pose;
            }
    );

    auto other_pose_sub = nh.subscribe<turtlesim::Pose>(
            nh.param<std::string>("other_name", "turtle1") + "/pose", 1,
            [&loc = env.other](const turtlesim::PoseConstPtr &pose) {
                loc = *pose;
            }
    );

    auto vel_pub = nh.advertise<geometry_msgs::Twist>(
            nh.param<std::string>("name", "mpc_turtle") + "/cmd_vel",
            1);


    geometry_msgs::Twist vel;

    ros::Duration(1.0).sleep(); // Lazy wait for first message
    ros::spinOnce();


    double dt = 0.5;
    auto rate = ros::Rate(1.0 / dt);

    mpc_simple::MPC mpc;

    const auto L = 1.0; // Distance between both axles (bicycle model)

    double v_rear = 0, alpha = 0;

    turtlesim::Pose goal;
    goal.x = 10.0;
    goal.y = 5;

    while (ros::ok()) {
        if ((pow(env.self.x - goal.x, 2) + pow(env.self.y - goal.y, 2)) < 1.0) {
            ROS_INFO("Reached Goal!");
            ROS_INFO("Shutting Down");
            vel.linear.x = 0;
            vel.linear.y = 0;
            vel.angular.z = 0;
            vel_pub.publish(vel);
            break;
        }
        ROS_INFO("Running");
        ROS_INFO("===> State: (%f, %f) ", v_rear, alpha);

        auto inputs = mpc.get_control_input(
                {v_rear, alpha,
                 env.self.x, env.self.y, env.self.theta,
                 env.other.x, env.other.y, goal.x, goal.y}
        );

        ROS_INFO("===> Controls: (%f, %f) ", inputs.a, inputs.alpha_dot);


        // Assume Rear wheel drive
        v_rear += dt * inputs.a;
        alpha += dt * inputs.alpha_dot;
        vel.linear.x = v_rear;
        vel.linear.y = v_rear * tan(alpha) / 2;
        vel.angular.z = v_rear * tan(alpha) / L;

        vel_pub.publish(vel);

        rate.sleep();
        ros::spinOnce();
    }
}