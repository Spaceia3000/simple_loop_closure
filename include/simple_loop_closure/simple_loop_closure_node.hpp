#pragma once

#define PCL_NO_PRECOMPILE

#include <rclcpp/rclcpp.hpp>
#include <simple_loop_closure/deterministic_loop_schedule.hpp>
#include <simple_loop_closure/fast_livo2_contract.hpp>
#include <simple_loop_closure/keyframe_gate.hpp>
#include <simple_loop_closure/stream_audit.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#include <nanoflann.hpp>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <vector>
#include <deque>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <chrono>
#include <condition_variable>

class SimpleLoopClosureNode : public rclcpp::Node
{
private: 
    typedef pcl::PointXYZI PointType;
    typedef pcl::PointCloud<PointType> PointCloudType;
    typedef std::lock_guard<std::mutex> MtxLockGuard;
    typedef std::shared_ptr<Eigen::Affine3d> Affine3dPtr;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 3> KDTreeMatrix;
    typedef nanoflann::KDTreeEigenMatrixAdaptor<KDTreeMatrix, 3, nanoflann::metric_L2_Simple> KDTree;
#if NANOFLANN_VERSION >= 0x150
    typedef std::vector<nanoflann::ResultItem<long int, double>> NanoFlannSearchResult;
#else
    typedef std::vector<std::pair<long int, double>> NanoFlannSearchResult;
#endif
    typedef std::pair<int, int> LoopEdgeID;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vis_pose_graph_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_pgo_odometry_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_original_keyframes_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_optimized_keyframes_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_loop_edges_;


    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cloud_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom_;
    message_filters::Connection cloud_audit_connection_;
    message_filters::Connection odom_audit_connection_;
    std::shared_ptr<Sync> synchronizer_;

    struct StreamAuditContext
    {
      std::mutex mutex;
      simple_loop_closure::StreamAudit audit;
    };

    bool stream_audit_enabled_;
    std::shared_ptr<StreamAuditContext> stream_audit_context_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stream_audit_service_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_req_;

    std_msgs::msg::ColorRGBA odom_edge_color_;
    std_msgs::msg::ColorRGBA loop_edge_color_;
    std_msgs::msg::ColorRGBA node_color_;
    double edge_scale_;
    double node_scale_;

    std::mutex mtx_buf_;
    std::deque<PointCloudType::Ptr> keyframes_cloud_;
    std::deque<Affine3dPtr> keyframes_odom_;
    std::deque<PointCloudType::Ptr> keyframes_cloud_copied_;
    std::deque<Affine3dPtr> keyframes_odom_copied_;
    std::deque<double> trajectory_dist_;
    std::deque<double> trajectory_dist_copied_;
    std::string odom_frame_id_;
    std_msgs::msg::Header latest_input_header_;
    simple_loop_closure::FastLivo2Contract input_contract_;

    std::unique_ptr<simple_loop_closure::DeterministicLoopSchedule> loop_schedule_;

    gtsam::ISAM2 isam2_;
    std::mutex mtx_res_;
    gtsam::Values optimization_result_;
    std::deque<LoopEdgeID> loop_edges_;

    gtsam::SharedNoiseModel prior_noise_;
    gtsam::SharedNoiseModel odom_noise_;
    gtsam::SharedNoiseModel const_loop_edge_noise_;

    pcl::IterativeClosestPoint<PointType, PointType> icp_;
    pcl::VoxelGrid<PointType> vg_target_;
    pcl::VoxelGrid<PointType> vg_source_;
    pcl::VoxelGrid<PointType> vg_map_;

    bool mapped_cloud_;
    double time_stamp_tolerance_;
    double keyframe_dist_th_;
    double keyframe_angular_dist_th_;
    double loop_search_time_diff_th_;
    double loop_search_dist_diff_th_;
    double loop_search_angular_dist_th_;
    int loop_search_frame_interval_;
    double search_radius_;
    int target_frame_num_;
    double target_voxel_leaf_size_;
    double source_voxel_leaf_size_;
    double vis_map_voxel_leaf_size_;
    double fitness_score_th_;
    int vis_map_cloud_frame_interval_;

    bool inputContractValid(
        const sensor_msgs::msg::PointCloud2 &cloud_msg,
        const nav_msgs::msg::Odometry &odom_msg) const;

    bool cloudSchemaValid(const sensor_msgs::msg::PointCloud2 &cloud_msg) const;

    std::atomic<bool> stop_loop_closure_thread_;
    std::atomic<bool> stop_visualize_thread_;
    std::atomic<bool> shutdown_started_{false};
    std::atomic<bool> loop_worker_failed_{false};
    std::condition_variable keyframe_changed_;

    std::thread loop_close_thread_;
    std::thread visualize_thread_;
    std::thread save_thread_;
    std::atomic<bool> saving_;
    std::string save_directory_;

    void initialize();

    simple_loop_closure::StreamAuditSnapshot captureStreamAuditSnapshot();

    std::string makeStreamAuditReport(const char* report_kind);

    void emitStreamAudit(const char* report_kind);

    bool saveEachFrames();

    void saveThread();

    void saveRequestCallback(const std_msgs::msg::String::ConstSharedPtr &directory);

    void publishPoseGraphOptimizedOdometry(const Eigen::Affine3d &affine_curr, const nav_msgs::msg::Odometry &odom_msg);

    void pointCloudAndOdometryCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr constructPointCloudMap(const int interval = 0);
    // PointCloudType::Ptr constructPointCloudMap(const int interval = 0);

    void publishMapCloud(const PointCloudType::Ptr &map_cloud);

    void constructVisualizationOdometryEdges(const int &res_size, const std_msgs::msg::Header &header, visualization_msgs::msg::Marker &marker_msg);

    void constructVisualizationOriginalEdges(const int &res_size, const std_msgs::msg::Header &header, visualization_msgs::msg::Marker &marker_msg);

    void constructVisualizationLoopEdges(const int &res_size, const std_msgs::msg::Header &header, visualization_msgs::msg::Marker &marker_msg);

    void constructVisualizationNodes(const int &res_size, const std_msgs::msg::Header &header, visualization_msgs::msg::Marker &marker_msg);

    void publishVisualizationGraph();

    void visualizeThread();

    void copyKeyFrames();

    void getLastOptimizedPose(Eigen::Affine3d &pose, int &id);

    bool constructNextOdometryGraph(gtsam::NonlinearFactorGraph &graph, gtsam::Values &init_estimate);

    bool snapshotOptimizedPrefix(std::size_t prefix_size, std::vector<Eigen::Affine3d> &poses);

    bool constructKDTreeMatrix(const std::vector<Eigen::Affine3d> &poses, KDTreeMatrix &kdtree_mat);

    int searchTarget(const KDTree &kdtree, const int &id_query, const Eigen::Affine3d &pose_query,
                     const rclcpp::Time &stamp_query, const std::vector<Eigen::Affine3d> &poses);

    pcl::PointCloud<pcl::PointXYZI>::Ptr constructTargetCloud(
        const int target_id, const std::vector<Eigen::Affine3d> &poses);

    pcl::PointCloud<pcl::PointXYZI>::Ptr constructSourceCloud(const int source_id);

    bool tryRegistration(const Eigen::Affine3d &init_pose, const PointCloudType::Ptr &source_cloud, const PointCloudType::Ptr &target_cloud, Eigen::Affine3d &result, double &score);

    bool constructLoopEdge(const std::size_t source_id, gtsam::NonlinearFactorGraph &graph,
                           LoopEdgeID &accepted_edge);

    bool updateISAM2(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &init_estimate);

    void processAvailableKeyFrames();

    void publishDeterminismAudit();

    void shutdownAndDrain();

    void loopCloseThread();

public:
    typedef std::shared_ptr<SimpleLoopClosureNode> Ptr;

    SimpleLoopClosureNode();

    ~SimpleLoopClosureNode();

    void spin();
};
