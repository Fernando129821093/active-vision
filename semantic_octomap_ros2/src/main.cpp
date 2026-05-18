#include <rclcpp/rclcpp.hpp>
#include <semantic_octomap_node/octomap_generator_ros2.h>
#include <cstdio>

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapGeneratorNode>(rclcpp::NodeOptions());

    // SingleThreadedExecutor — service serializes behind insertCloudCallback
    // but at least the wire-format shift bug should not occur.
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
