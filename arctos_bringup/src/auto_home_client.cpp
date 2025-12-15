// arctos_bringup/src/auto_home_client.cpp
/*
You (or auto_home_client) call:

    /go_home → which calls homeAllSync() in the hardware interface, using homeSync() + setZeroSync() from the driver.

    optional /dump_motors → prints full state of all motors

    optional /wait_all_converged → ensures all commanded moves are finished

MoveIt plans and executes trajectories against a homed, synchronized robot.
*/
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class AutoHomeClient : public rclcpp::Node
{
public:
    AutoHomeClient()
        : Node("auto_home_client")
    {
        client_ = this->create_client<std_srvs::srv::Trigger>("go_home");
        wait_client_and_call();
    }

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;

    void wait_client_and_call()
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for /go_home service...");
        }

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client_->async_send_request(req);

        if (future.wait_for(std::chrono::seconds(60)) == std::future_status::ready)
        {
            auto res = future.get();
            if (res->success)
            {
                RCLCPP_INFO(this->get_logger(), "Homing succeeded: %s", res->message.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Homing failed: %s", res->message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Homing request timed out");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoHomeClient>());
    rclcpp::shutdown();
    return 0;
}
