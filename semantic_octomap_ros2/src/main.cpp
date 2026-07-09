#include <rclcpp/rclcpp.hpp>
#include <semantic_octomap_node/octomap_generator_ros2.h>
#include <cstdio>

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapGeneratorNode>(rclcpp::NodeOptions());

    // MultiThreadedExecutor: the cloud-insert subscription and the query_rle
    // service live in separate callback groups (see constructor). Under the old
    // SingleThreadedExecutor the continuous query_rle load starved the cloud
    // callback, so the octree never ingested clouds (frozen semantic obs).
    // Octree access is serialized by octree_mutex_, so 3 threads are safe.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
