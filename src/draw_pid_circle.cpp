#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "error_msgs/msg/error.hpp"

double er_sum = 0;
double etheta_sum =0;
double er_prev = 0;
double etheta_prev =0;

class TurtleFbControl : public rclcpp::Node // MODIFY NAME
{
public:
    TurtleFbControl() : Node("turtle_fb_control") // MODIFY NAME
    {
        RCLCPP_INFO(this->get_logger(), "turtle control has been started");
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel",10);
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleFbControl::callbackPose, this, std::placeholders::_1));
        
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&TurtleFbControl::controlLoop, this));
        
    }

private:

    void callbackPose(const turtlesim::msg::Pose::SharedPtr pose)
    {   //get Pose msg
        pose_ = *pose.get();
        RCLCPP_INFO(this->get_logger(), "Received pose: x=%f, y=%f, theta=%f", pose_.x, pose_.y, pose_.theta);
    }

    void controlLoop()
    {
        auto msg = geometry_msgs::msg::Twist();

        //PID control gain
        double Kp = 1;
        double Ki = 0;
        double Kd = 0;
        double u_r =0;
        double u_theta =0;
        //simple reference point
        double xr = 30;
        double yr = 30;
        double theta_r = 0;
        //circle trajectory
        
        //get error
        auto e_r = sqrt(pow(xr - pose_.x,2) + pow(yr - pose_.y,2));
        auto e_theta = theta_r - pose_.theta;

        //derive control signal u (with PID control)
        if (e_r > 0)
        {
            double dedt_r = (e_r - er_prev)/0.01;
            double dedt_theta = (e_theta - etheta_prev)/0.01;
            er_sum = er_sum + e_r * 0.01;
            etheta_sum = etheta_sum + e_theta * 0.01;
            u_r = Ki*e_r + Ki*er_sum + Kd*dedt_r;
            u_theta = Kp*e_theta + Ki*etheta_sum + Kd*dedt_theta;
        }
        else
        {
            // target reached!
            msg.linear.x = 0;
            msg.angular.z = 0;
        }
        
        msg.linear.x = u_r;
        msg.angular.z = u_theta;
        cmd_vel_publisher_->publish(msg);
        er_prev = e_r;
        etheta_prev = e_theta;
        // this->publishCmdVel(u_r, u_theta);
    }
    void publishCmdVel(double x, double theta)
    {   //publish control signal u
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = x;
        msg.angular.z = theta;
        cmd_vel_publisher_->publish(msg);
    }

    std::string name_;
    turtlesim::msg::Pose pose_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleFbControl>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}