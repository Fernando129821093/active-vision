#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/empty.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <semantic_octomap_node/octomap_generator.h>
#include <semantic_octomap_interfaces/srv/get_rle.hpp>

#include <string>
#include <memory>

class OctomapGeneratorNode : public rclcpp::Node
{
public:
    explicit OctomapGeneratorNode(const rclcpp::NodeOptions& options);
    ~OctomapGeneratorNode();

private:
    void reset();
    void insertCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
    void publishMaps(const std_msgs::msg::Header& header);
    void publishMarkers(const std_msgs::msg::Header& header);

    void toggleUseSemanticColor(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);

    void queryRLE(
        const std::shared_ptr<semantic_octomap_interfaces::srv::GetRLE::Request> req,
        std::shared_ptr<semantic_octomap_interfaces::srv::GetRLE::Response> res);

    OctomapGeneratorBase<SemanticOctree>* octomap_generator_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_color_service_;
    rclcpp::Service<semantic_octomap_interfaces::srv::GetRLE>::SharedPtr rle_service_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr fullmap_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr colormap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    // Dedicated callback group so GetRLE service calls run in parallel with
    // the cloud insert subscription (otherwise they queue up to ~170 ms each).
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    // Octree is NOT thread-safe — serialize reads/writes between the cloud
    // insert callback (writer) and GetRLE service handler (reader).
    std::mutex octree_mutex_;
    // TF only needed when cloud frame != world frame; kept for future use
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    unsigned int clouds_received_{0};

    std::string world_frame_id_;
    std::string pointcloud_topic_;
    double max_range_, raycast_range_;
    double clamping_thres_max_, clamping_thres_min_;
    double psi_, phi_, resolution_, occupancy_thres_;
    double prob_hit_, prob_miss_;
    bool publish_2d_map_;
};
