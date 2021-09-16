#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/get_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <unistd.h>


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
            camera_topic_, 10, std::bind(&GetImageNode::callbackImageSub, this, _1));
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10); ///camera/image_raw
        get_image_server_ = this->create_service<custom_interfaces::srv::GetImage>(
            "get_image",
            std::bind(&GetImageNode::callbackGetImage, this, _1, _2));
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
        // republish image msg with lower frequency
        sleep(1.0);
        image.data = img->data;
        image.encoding = img->encoding;
        image.header = img->header;
        image.height = img->height;
        image.is_bigendian = img->is_bigendian;
        image.width = img->width;
        image.step = img->step;
        image_publisher_->publish(image);
    }

    rclcpp::Service<custom_interfaces::srv::GetImage>::SharedPtr get_image_server_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    sensor_msgs::msg::Image::SharedPtr img;
    sensor_msgs::msg::Image image;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GetImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}