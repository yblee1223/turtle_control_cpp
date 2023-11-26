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

using namespace std::chrono_literals;
using namespace std;

double t(0.0);

double xdot(0.0);
double ydot(0.0);
double xddot(0.0);
double yddot(0.0);

double a(3.0);
double b(3.0);
double dt(0.1);

const char *msg = R"(
    ------------------------------------
    draw circle using turtlesim
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

class CircleTrajectory : public rclcpp::Node{
    public:
        CircleTrajectory() : Node("circle_trajectory")
        {
            printf("%s", msg);
            // define msg
            geometry_msgs::msg::Twist twist;

            // about Templete < > 
            // because you are using template
            // template is a generic type that can be used for many different types of functions
            pub_ = this->create_publisher<geometry_msgs::msg::Twist> ("/turtle1/cmd_vel", 10); // template
            
            // define service
            rst_ = this->create_client<std_srvs::srv::Empty> ("/clear"); // rst_ is a client that requests a service
            setp_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute"); // setp_ is a client that requests a service
            auto pos = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>(); // pos is a shared pointer that points to a TeleportAbsolute::Request object

            while(!setp_ -> wait_for_service(1s)){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            // init pos
            pos->x = 5.54;
            pos->y = 2.54;
            pos->theta = 0.0;
            setp_->async_send_request(pos);
            rst_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());

            timer_ = this->create_wall_timer(
                100ms, std::bind(&CircleTrajectory::timer_callback, this)
            );
        }

    private:
        // for timer
        void timer_callback()
        {
            if(key == 's')
            {
                auto command = geometry_msgs::msg::Twist();
                xdot = -a * b * sin(b * t);
                ydot = a * b * cos(b * t);
                command.linear.x = sqrt(pow(xdot, 2.0) + pow(ydot, 2.0));

                xddot = -a * pow(b, 2.0) * cos(b * t);
                yddot = -a * pow(b, 2.0) * sin(b * t);
                command.angular.z = (yddot * xdot - ydot * xddot) / (pow(xdot, 2.0) + pow(ydot, 2.0));

                pub_->publish(command);
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
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr rst_;
        rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr setp_;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<CircleTrajectory>();
    auto get_k = std::make_shared<GetKeyNode>();

    executor.add_node(node);
    executor.add_node(get_k);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}







