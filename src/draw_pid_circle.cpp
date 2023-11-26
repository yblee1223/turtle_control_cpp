#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

double er_sum = 0;
double etheta_sum =0;
double er_prev = 0;
double etheta_prev =0;

float feed_x(0.0);
float feed_y(0.0);
float feed_th(0.0);

double init_x(5.54445);
double init_y(5.54445);
double init_th(0.0);

double x(0.0);
double y(0.0);
double th(0.0);
double xdot(0.0);
double ydot(0.0);
double xddot(0.0);
double yddot(0.0);

double t = 0;

double a = 1;
double b = 1;

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
    //PID contorl gain
    double Kp_r = 1;
    double Kd_r = 0;
    double Ki_r = 0;

    double Kp_th = 2;
    double Kd_th = 0;
    double Ki_th = 1;
    
    //control signal
    double u_r = 0;
    double u_theta = 0;
    //simpler reference point
    double xr = 9;
    double yr = 7;

    double theta_r = 0;
    double e_r = 1;
    double e_theta = 1;

    double dt = 0.0001;
    //circle trajectory

    void callbackPose(const turtlesim::msg::Pose::SharedPtr pose)
    {   //get Pose msg
        pose_ = *pose.get();
        //RCLCPP_INFO(this->get_logger(), "Received pose: x=%f, y=%f, theta=%f", pose_.x, pose_.y, pose_.theta);
    }

    void controlLoop()
    {
        auto msg = geometry_msgs::msg::Twist();

        x = init_x + a * sin(b * t);
        y = init_y + a * cos(b * t);

        // loss function
        e_r = sqrt(pow(x - pose_.x, 2) + pow(y - pose_.y, 2));
        theta_r = atan2((y - pose_.y) , (x - pose_.x));
        e_theta = theta_r - pose_.theta;

        //derive control signal u (with PID control)
        
        double dedt_r = (e_r - er_prev) / dt;
        double dedt_theta = (e_theta - etheta_prev) / dt;
        er_sum = er_sum + e_r * dt;
        etheta_sum = etheta_sum + e_theta * dt;
        u_r = Kp_r * e_r + Ki_r * er_sum + Kd_r * dedt_r;
        u_theta = Kp_th * e_theta + Ki_th * etheta_sum + Kd_th * dedt_theta;
        

        RCLCPP_INFO(this->get_logger(), "Received pose: e_r=%f, e_theta=%f",e_r, e_theta);
        RCLCPP_INFO(this->get_logger(), "Received pose: u_r=%f, u_theta=%f",u_r, u_theta);

        msg.linear.x = u_r;
        msg.angular.z = u_theta;
        cmd_vel_publisher_->publish(msg);
        er_prev = e_r;
        etheta_prev = e_theta;

        t = t + dt;
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