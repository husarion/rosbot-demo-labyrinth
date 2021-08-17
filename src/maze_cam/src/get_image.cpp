#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/get_image.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class GetImageNode : public rclcpp::Node
{
public:
    GetImageNode() : Node("get_image_server")
    {   
        this->declare_parameter("camera_topic", "image_raw");
        std::string camera_topic_ = this->get_parameter("camera_topic").as_string();

        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 10, std::bind(&GetImageNode::callbackImageSub, this, _1)
        );
        get_image_server_ = this->create_service<custom_interfaces::srv::GetImage>(
            "get_image",
            std::bind(&GetImageNode::callbackGetImage, this,_1,_2));
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }

private:
    void callbackGetImage(const custom_interfaces::srv::GetImage::Request::SharedPtr request,
                            const custom_interfaces::srv::GetImage::Response::SharedPtr response)
    {
        response->image.data = img->data;
        response->image.encoding = img->encoding;
        response->image.header = img->header;
        response->image.height = img->height;
        response->image.is_bigendian = img->is_bigendian;
        response->image.width = img->width;
        response->image.step = img->step;
    }

    void callbackImageSub(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        img = msg;
    }

    rclcpp::Service<custom_interfaces::srv::GetImage>::SharedPtr get_image_server_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    sensor_msgs::msg::Image::SharedPtr img;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GetImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}