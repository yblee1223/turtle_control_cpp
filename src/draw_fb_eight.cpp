#include <stdio.h>
#include <unistd.h> // unistd.h is a header file that provides access to the POSIX operating system API
#include <termios.h> // termios.h is the header file that provides the definitions for the termios structure
#include <map> // map is a container that stores elements in key-value pairs where keys are unique, and multiple keys can map to the same value
#include <functional> // functional is a header that contains a namespace with various class templates that wrap and define function objectsb
#include <string> // string is a header that defines several functions to manipulate C++ strings
#include <chrono> // chrono is a header that contains functions and classes for time measurement
#include <memory> // memory is a header that defines general utilities to manage dynamic memory
#include <thread> // thread is a header that defines a class with the ability to represent a thread of execution

#include "rclcpp/rclcpp.hpp" // rclcpp/rclcpp.hpp is a header that includes all the necessary headers to use the ROS Client Library for C++
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "turtlesim/srv/teleport_absolute.hpp" 
#include "turtlesim/msg/pose.hpp"
#include "error_msgs/msg/error.hpp"

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

double t(0.0);

float feed_x(0.0);
float feed_y(0.0);
float feed_th(0.0);

// double init_x(5.54445);
// double init_y(5.54445);
// double init_th(-20.0);

double init_x(10);
double init_y(10);
double init_th(180);

double x(0.0);
double y(0.0);
double th(0.0);
double xdot(0.0);
double ydot(0.0);
double xddot(0.0);
double yddot(0.0);

double k1(10.0);
double k2(10.0);
double k3(10.0);

double a(5.0);
double b(0.5);
double dt(0.001);

const char *msg = R"(
    ------------------------------------
    draw circle using turtlesim (with feedback control)
    1. run node
    2. press 's' -> start
    3. press spacebar -> stop
)";

char key = ' ';


int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // store old setting, copy to new setting
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO); // clear ICANON and ECHO
    newt.c_iflag |= IGNBRK; // ignore break condition
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF); // disable software flow control
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN); // raw mode
    newt.c_cc[VMIN] = 1; // minimum number of characters to read
    newt.c_cc[VTIME] = 0; // time to wait for data (tenths of seconds)
    tcsetattr(fileno(stdin), TCSANOW, &newt); // set new terminal setting

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old terminal setting

    return ch;
}

class GetKeyNode : public rclcpp::Node{
    public:
        GetKeyNode() : Node("get_Key")
        {
            timer_get = this->create_wall_timer(
                200ms, std::bind(&GetKeyNode::timer_callback_get, this) // bind timer_callback_get to this
            );
        }
    private:
        void timer_callback_get()
        {
            key = getch();
        }
        rclcpp::TimerBase::SharedPtr timer_get; // timer for get key
};

class feedbackEight : public rclcpp::Node{
    public:
        feedbackEight() : Node("feedback_eight")
        {
            printf("%s", msg);
            // define msg
            geometry_msgs::msg::Twist twist;

            pub_ = this->create_publisher<geometry_msgs::msg::Twist> ("/turtle1/cmd_vel", 10); // template
            pub_error = this->create_publisher<error_msgs::msg::Error> ("/error", 10);
            sub_ = this->create_subscription<turtlesim::msg::Pose> ("/turtle1/pose", 10, std::bind(&feedbackEight::feedback_callback, this, _1));

            
            rst_ = this->create_client<std_srvs::srv::Empty> ("/clear"); 
            setp_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute"); 
            auto pos = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>(); 

            while(!setp_ -> wait_for_service(1s)){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            // init pos
            pos->x = init_x;
            pos->y = init_y;
            pos->theta = init_th;

            x = pos->x;
            y = pos->y;
            th = pos->theta;

            setp_->async_send_request(pos);

            while(!rst_ -> wait_for_service(1s)){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            auto req = std::make_shared<std_srvs::srv::Empty::Request>();
            rst_->async_send_request(req);

            // define timer
            timer_ = this->create_wall_timer(
                1ms, std::bind(&feedbackEight::timer_callback, this)
            );
        }

    private:
        // for timer
        void timer_callback()
        {
            if(key == 's')
            {
                auto command = geometry_msgs::msg::Twist();
                auto loss = error_msgs::msg::Error();

                xdot = a * b * cos(b * t);
                ydot = a * b * cos(b * t) * cos(b * t) - a * b * sin(b * t) * sin(b * t);
                double lin_x = sqrt(pow(xdot, 2.0) + pow(ydot, 2.0));

                x = init_x + a * sin(b * t);
                y = init_y + a * sin(b * t) * cos(b * t);
                th = atan2(ydot, xdot);

                double e1 = cos(th) * (x - feed_x) + sin(th) * (y - feed_y);
                double e2 = -sin(th) * (x - feed_x) + cos(th) * (y - feed_y);
                double e3 = th - feed_th;

                xddot = -a * pow(b, 2.0) * sin(b * t);
                yddot = -4 * a * pow(b, 2.0) * sin(b * t) * cos(b * t);
                double ang_z = (yddot * xdot - ydot * xddot) / (pow(xdot, 2.0) + pow(ydot, 2.0));

                command.linear.x = lin_x * cos(e3) + k1 * e1;
                command.angular.z = ang_z + k2 * lin_x * e2 + k3 * lin_x * sin(e3);

                loss.x = e1;
                loss.y = e2;
                loss.theta = e3;
                loss.r = sqrt(pow(e1, 2.0) + pow(e2, 2.0));

                pub_->publish(command);
                pub_error->publish(loss);
                t = t + dt;
            }
            else if(key == '\x03'){
                auto stop = geometry_msgs::msg::Twist();

                stop.linear.x = 0.0;
                stop.angular.z = 0.0;
                pub_->publish(stop);

                timer_->cancel();
                rclcpp::shutdown();
            }
            else if(key == ' ')
            {
                auto stop = geometry_msgs::msg::Twist();

                stop.linear.x = 0.0;
                stop.angular.z = 0.0;
                pub_->publish(stop);
            }
        }
        void feedback_callback(const turtlesim::msg::Pose::SharedPtr data) const
        {
            feed_x = data->x;
            feed_y = data->y;
            feed_th = data->theta;
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr rst_;
        rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr setp_;
        rclcpp::Publisher<error_msgs::msg::Error>::SharedPtr pub_error;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<feedbackEight>();
    auto get_k = std::make_shared<GetKeyNode>();

    executor.add_node(node);
    executor.add_node(get_k);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}







