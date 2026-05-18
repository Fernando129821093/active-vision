#include <semantic_octomap_node/octomap_generator_ros2.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <iostream>
#include <cmath>

OctomapGeneratorNode::OctomapGeneratorNode(const rclcpp::NodeOptions& options)
: Node("semantic_octomap_node", options),
  octomap_generator_(nullptr)
{
    RCLCPP_INFO(this->get_logger(), "Semantic OctoMap generator (ROS2 Jazzy) starting...");

    // Declare parameters
    this->declare_parameter("pointcloud_topic",   std::string("/nbv/semantic_pointcloud"));
    this->declare_parameter("world_frame_id",     std::string("world"));
    this->declare_parameter("resolution",         0.05);
    this->declare_parameter("max_range",          10.0);
    this->declare_parameter("raycast_range",      10.0);
    this->declare_parameter("clamping_thres_min", 0.12);
    this->declare_parameter("clamping_thres_max", 0.97);
    this->declare_parameter("occupancy_thres",    0.5);
    this->declare_parameter("prob_hit",           0.7);
    this->declare_parameter("prob_miss",          0.4);
    this->declare_parameter("psi",                0.3);
    this->declare_parameter("phi",                -0.1);
    this->declare_parameter("publish_2d_map",     false);

    octomap_generator_ = new OctomapGenerator<PCLSemantics, SemanticOctree>();
    reset();

    // Publishers (latched via transient_local QoS)
    auto qos_latched = rclcpp::QoS(1).transient_local();
    fullmap_pub_  = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_full",  qos_latched);
    colormap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_color", qos_latched);
    occ_map_pub_  = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_map_2D", 1);
    marker_pub_   = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "semantic_markers", qos_latched);

    // Services — back to default callback group (single-threaded) to rule
    // out the ReentrantCallbackGroup as the cause of the wire-format shift bug.
    toggle_color_service_ = this->create_service<std_srvs::srv::Empty>(
        "toggle_use_semantic_color",
        std::bind(&OctomapGeneratorNode::toggleUseSemanticColor, this,
                  std::placeholders::_1, std::placeholders::_2));

    rle_service_ = this->create_service<semantic_octomap_interfaces::srv::GetRLE>(
        "query_rle",
        std::bind(&OctomapGeneratorNode::queryRLE, this,
                  std::placeholders::_1, std::placeholders::_2));

    // TF2 buffer (no listener thread — avoids executor deadlock when TF is empty)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    // Direct subscription with depth=10 (system default QoS: RELIABLE, VOLATILE, KEEP_LAST)
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, 10,
        std::bind(&OctomapGeneratorNode::insertCloudCallback, this, std::placeholders::_1));

    // NOTE: diag_timer removed — count_publishers/count_subscribers from a wall_timer
    // appears to deadlock the executor with FastDDS in this configuration

    RCLCPP_INFO(this->get_logger(),
        "Listening on '%s', world frame '%s', resolution %.3f m, NUM_SEMANTICS=%d",
        pointcloud_topic_.c_str(), world_frame_id_.c_str(), resolution_, NUM_SEMANTICS);
}

OctomapGeneratorNode::~OctomapGeneratorNode()
{
    delete octomap_generator_;
}

void OctomapGeneratorNode::reset()
{
    this->get_parameter("pointcloud_topic",   pointcloud_topic_);
    this->get_parameter("world_frame_id",     world_frame_id_);
    this->get_parameter("resolution",         resolution_);
    this->get_parameter("max_range",          max_range_);
    this->get_parameter("raycast_range",      raycast_range_);
    this->get_parameter("clamping_thres_min", clamping_thres_min_);
    this->get_parameter("clamping_thres_max", clamping_thres_max_);
    this->get_parameter("occupancy_thres",    occupancy_thres_);
    this->get_parameter("prob_hit",           prob_hit_);
    this->get_parameter("prob_miss",          prob_miss_);
    this->get_parameter("psi",                psi_);
    this->get_parameter("phi",                phi_);
    this->get_parameter("publish_2d_map",     publish_2d_map_);

    octomap_generator_->setResolution(static_cast<float>(resolution_));
    octomap_generator_->setMaxRange(static_cast<float>(max_range_));
    octomap_generator_->setRayCastRange(static_cast<float>(raycast_range_));
    octomap_generator_->setClampingThresMin(static_cast<float>(clamping_thres_min_));
    octomap_generator_->setClampingThresMax(static_cast<float>(clamping_thres_max_));
    octomap_generator_->setOccupancyThres(static_cast<float>(occupancy_thres_));
    octomap_generator_->setProbHit(static_cast<float>(prob_hit_));
    octomap_generator_->setProbMiss(static_cast<float>(prob_miss_));
    octomap_generator_->setPsi(static_cast<float>(psi_));
    octomap_generator_->setPhi(static_cast<float>(phi_));
}

void OctomapGeneratorNode::insertCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
{
    ++clouds_received_;
    // Throttled progress log every 100 clouds (otherwise spams at ~3 Hz).
    if (clouds_received_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
            "[insertCloudCallback] cloud #%u (%u pts)",
            clouds_received_, cloud_msg->width * cloud_msg->height);
    }

    Eigen::Matrix4f sensorToWorld = Eigen::Matrix4f::Identity();

    // Only do TF lookup when source frame differs from world frame
    if (cloud_msg->header.frame_id != world_frame_id_) {
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_->lookupTransform(
                world_frame_id_, cloud_msg->header.frame_id,
                cloud_msg->header.stamp,
                std::chrono::milliseconds(200));
            Eigen::Isometry3d iso = tf2::transformToEigen(tf_stamped);
            sensorToWorld = iso.matrix().cast<float>();
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(),
                "TF lookup '%s'->'%s' failed: %s — skipping cloud",
                cloud_msg->header.frame_id.c_str(), world_frame_id_.c_str(), ex.what());
            return;
        }
    }

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        octomap_generator_->insertPointCloud(cloud, sensorToWorld);
    }
    publishMaps(cloud_msg->header);  // re-enabled for markers
}

void OctomapGeneratorNode::publishMaps(const std_msgs::msg::Header& header)
{
    octomap_msgs::msg::Octomap map_msg;
    map_msg.header.frame_id = world_frame_id_;
    map_msg.header.stamp    = header.stamp;

    {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        octomap_generator_->setWriteSemantics(true);
        if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg))
            fullmap_pub_->publish(map_msg);
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to serialize full semantic octomap");
    }
    // NOTE: skipping second fullMapToMsg(color) — caused memory corruption.
    // The semantic map already encodes class colors via setWriteSemantics(true).
    publishMarkers(header);
}

void OctomapGeneratorNode::publishMarkers(const std_msgs::msg::Header& header)
{
    auto* tree = octomap_generator_->getOctree();
    if (!tree) return;

    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = world_frame_id_;
    m.header.stamp    = header.stamp;
    m.ns     = "semantic_octomap";
    m.id     = 0;
    m.type   = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = resolution_;
    m.scale.y = resolution_;
    m.scale.z = resolution_;
    m.color.a = 1.0f;  // required when using per-point colors

    {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
            if (!tree->isNodeOccupied(*it)) continue;
            if (!it->isSemanticsSet()) continue;

            geometry_msgs::msg::Point p;
            p.x = it.getX();
            p.y = it.getY();
            p.z = it.getZ();
            m.points.push_back(p);

            auto c = it->getSemantics().getSemanticColor();
            std_msgs::msg::ColorRGBA rgba;
            rgba.r = c.r / 255.0f;
            rgba.g = c.g / 255.0f;
            rgba.b = c.b / 255.0f;
            rgba.a = 0.85f;
            m.colors.push_back(rgba);
        }
    }

    if (!m.points.empty())
        ma.markers.push_back(m);
    marker_pub_->publish(ma);
}

void OctomapGeneratorNode::toggleUseSemanticColor(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    bool current = octomap_generator_->isUseSemanticColor();
    octomap_generator_->setUseSemanticColor(!current);
    RCLCPP_INFO(this->get_logger(), "use_semantic_color toggled to %s",
                !current ? "true" : "false");
}

void OctomapGeneratorNode::queryRLE(
    const std::shared_ptr<semantic_octomap_interfaces::srv::GetRLE::Request> req,
    std::shared_ptr<semantic_octomap_interfaces::srv::GetRLE::Response> res)
{
    const octomap::point3d origin(
        static_cast<float>(req->origin.x),
        static_cast<float>(req->origin.y),
        static_cast<float>(req->origin.z));

    // Simplified: instead of get_ray_RLE (which crashes on populated trees),
    // just query the endpoint voxel directly. We lose along-ray info but
    // get stable per-endpoint semantic data.
    {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        auto* tree = octomap_generator_->getOctree();
        for (const auto& ep : req->end_points) {
            semantic_octomap_interfaces::msg::RayRLE rayRLE_msg;
            semantic_octomap_interfaces::msg::LE le_msg;
            le_msg.le.push_back(1.0);  // run length

            auto* node = tree
                ? tree->search(octomap::point3d(
                      static_cast<float>(ep.x),
                      static_cast<float>(ep.y),
                      static_cast<float>(ep.z)))
                : nullptr;
            if (node && node->isSemanticsSet()) {
                auto sem = node->getSemantics();
                for (int i = 0; i < NUM_SEMANTICS; ++i)
                    le_msg.le.push_back(sem.data[i].logOdds);
                le_msg.le.push_back(sem.others);
            } else if (node) {
                // Node exists but no semantics set yet — uniform unknown
                float l = node->getLogOdds()
                        - std::log(static_cast<float>(NUM_SEMANTICS + 1));
                for (int c = 0; c < NUM_SEMANTICS + 1; ++c)
                    le_msg.le.push_back(l);
            } else {
                // No node at this voxel — pure unknown
                for (int c = 0; c < NUM_SEMANTICS + 1; ++c)
                    le_msg.le.push_back(-0.1f);
            }
            rayRLE_msg.le_list.push_back(le_msg);
            res->rle_list.push_back(rayRLE_msg);
        }
    }
}
