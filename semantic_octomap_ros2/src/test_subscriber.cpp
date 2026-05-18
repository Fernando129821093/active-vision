// Test that mimics OctomapGeneratorNode patterns to find what breaks
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <semantic_octomap_interfaces/srv/get_rle.hpp>
#include <tf2_ros/buffer.h>
#include <semantic_octomap_node/octomap_generator.h>
#include <cstdio>

class TestNode : public rclcpp::Node
{
public:
    TestNode() : Node("test_node_class")
    {
        // Mimic OctomapGeneratorNode: declare params
        declare_parameter("pointcloud_topic", std::string("/nbv/semantic_pointcloud"));
        declare_parameter("world_frame_id",   std::string("world"));
        declare_parameter("resolution",       0.05);
        declare_parameter("max_range",        10.0);

        std::string topic = get_parameter("pointcloud_topic").as_string();

        // Same components
        octomap_generator_ = new OctomapGenerator<PCLSemantics, SemanticOctree>();
        octomap_generator_->setResolution(0.05f);

        auto qos_latched = rclcpp::QoS(1).transient_local();
        fullmap_pub_  = create_publisher<octomap_msgs::msg::Octomap>("octomap_full",  qos_latched);
        colormap_pub_ = create_publisher<octomap_msgs::msg::Octomap>("octomap_color", qos_latched);
        marker_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>("semantic_markers", qos_latched);
        occ_map_pub_  = create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_map_2D", 1);

        toggle_svc_ = create_service<std_srvs::srv::Empty>("toggle_use_semantic_color",
            [](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>) {});
        rle_svc_ = create_service<semantic_octomap_interfaces::srv::GetRLE>("query_rle",
            [](const std::shared_ptr<semantic_octomap_interfaces::srv::GetRLE::Request>,
               std::shared_ptr<semantic_octomap_interfaces::srv::GetRLE::Response>) {});

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());

        // Use std::bind like the real node
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, 10,
            std::bind(&TestNode::cb, this, std::placeholders::_1));

        std::printf("[test-class] node ready, subscribed to '%s'\n", topic.c_str());
    }
    ~TestNode() { delete octomap_generator_; }

private:
    void cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        ++n_received_;
        std::printf("[test-class] cloud #%u received: %u pts\n",
            n_received_, msg->width * msg->height);
    }

    OctomapGeneratorBase<SemanticOctree>* octomap_generator_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr fullmap_pub_, colormap_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_svc_;
    rclcpp::Service<semantic_octomap_interfaces::srv::GetRLE>::SharedPtr rle_svc_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    unsigned int n_received_ = 0;
};

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
