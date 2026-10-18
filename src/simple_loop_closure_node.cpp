#include <simple_loop_closure/simple_loop_closure_node.hpp>
#include <simple_loop_closure/math_safety.hpp>

#include <csignal>

namespace
{

using SteadyClock = std::chrono::steady_clock;

std::uint64_t steadyNowNs() noexcept
{
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          SteadyClock::now().time_since_epoch()).count());
}

class ScopedSyncServiceTimer
{
public:
  explicit ScopedSyncServiceTimer(
      std::shared_ptr<simple_loop_closure::IngressTelemetry> telemetry) noexcept
  : telemetry_(std::move(telemetry)), start_ns_(telemetry_ ? steadyNowNs() : 0)
  {
  }

  ~ScopedSyncServiceTimer() noexcept
  {
    if (telemetry_)
      telemetry_->synchronized_callback_service.observe(steadyNowNs() - start_ns_);
  }

private:
  std::shared_ptr<simple_loop_closure::IngressTelemetry> telemetry_;
  std::uint64_t start_ns_;
};

volatile std::sig_atomic_t shutdown_signal_received = 0;

void requestStagedShutdown(int)
{
  shutdown_signal_received = 1;
}

std::string formatQos(const rclcpp::QoS& qos)
{
  const rmw_qos_profile_t& profile = qos.get_rmw_qos_profile();
  const char* reliability =
      profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE ? "reliable" :
      profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT ? "best_effort" :
      "unknown_reliability";
  const char* history =
      profile.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST ? "keep_last" :
      profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL ? "keep_all" :
      "unknown_history";

  std::ostringstream description;
  description << reliability << '_' << history;
  if (profile.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    description << '_' << profile.depth;
  return description.str();
}

}  // namespace

SimpleLoopClosureNode::SimpleLoopClosureNode() : Node("simple_loop_closure")
{
  initialize();
}

SimpleLoopClosureNode::~SimpleLoopClosureNode()
{
  shutdownAndDrain();
  if (save_thread_.joinable())
    save_thread_.join();
}

void SimpleLoopClosureNode::initialize()
{
  pub_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pgo_map_cloud", 1);
  pub_vis_pose_graph_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis_pose_graph", 1);
  pub_original_keyframes_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/simple_loop_closure/original_keyframes", 1);
  pub_optimized_keyframes_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/simple_loop_closure/optimized_keyframes", 1);
  pub_loop_edges_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/simple_loop_closure/loop_edges", 1);

  this->declare_parameter<bool>("mapped_cloud", true);
  this->declare_parameter<double>("time_stamp_tolerance", 0.01);
  this->declare_parameter<std::int64_t>("sync_queue_size", 50);
  this->declare_parameter<std::int64_t>("cloud_input_depth", 10);
  this->declare_parameter<std::int64_t>("pgo_output_depth", 1);
  this->declare_parameter<double>("keyframe_dist_th", 0.3);
  this->declare_parameter<double>("keyframe_angular_dist_th", 0.2);
  this->declare_parameter<double>("loop_search_time_diff_th", 30.0);
  this->declare_parameter<double>("loop_search_dist_diff_th", 30.0);
  this->declare_parameter<double>("loop_search_angular_dist_th", 3.14);
  this->declare_parameter<int>("loop_search_frame_interval", 1);
  this->declare_parameter<double>("search_radius", 15.0);
  this->declare_parameter<int>("target_frame_num", 50);
  this->declare_parameter<double>("target_voxel_leaf_size", 0.4);
  this->declare_parameter<double>("source_voxel_leaf_size", 0.4);
  this->declare_parameter<double>("vis_map_voxel_leaf_size", 0.8);
  this->declare_parameter<double>("fitness_score_th", 0.3);
  this->declare_parameter<int>("vis_map_cloud_frame_interval", 3);
  this->declare_parameter<double>("visualization.optimized_edge_width_m", 0.10);
  this->declare_parameter<double>("visualization.original_edge_width_m", 0.10);
  this->declare_parameter<double>("visualization.loop_edge_width_m", 0.10);
  this->declare_parameter<std::string>("expected_fixed_frame", "camera_init");
  this->declare_parameter<std::string>("expected_body_frame", "aft_mapped");
  this->declare_parameter<bool>("require_exact_timestamps", true);
  this->declare_parameter<bool>("simple_stream_audit", false);
  this->declare_parameter<bool>("cloud_input_telemetry", false);

  this->get_parameter("mapped_cloud", mapped_cloud_);
  this->get_parameter("time_stamp_tolerance", time_stamp_tolerance_);
  this->get_parameter("sync_queue_size", sync_queue_size_);
  this->get_parameter("cloud_input_depth", cloud_input_depth_);
  this->get_parameter("pgo_output_depth", pgo_output_depth_);
  this->get_parameter("keyframe_dist_th", keyframe_dist_th_);
  this->get_parameter("keyframe_angular_dist_th", keyframe_angular_dist_th_);
  this->get_parameter("loop_search_time_diff_th", loop_search_time_diff_th_);
  this->get_parameter("loop_search_dist_diff_th", loop_search_dist_diff_th_);
  this->get_parameter("loop_search_angular_dist_th", loop_search_angular_dist_th_);
  this->get_parameter("loop_search_frame_interval", loop_search_frame_interval_);
  this->get_parameter("search_radius", search_radius_);
  this->get_parameter("target_frame_num", target_frame_num_);
  this->get_parameter("target_voxel_leaf_size", target_voxel_leaf_size_);
  this->get_parameter("source_voxel_leaf_size", source_voxel_leaf_size_);
  this->get_parameter("vis_map_voxel_leaf_size", vis_map_voxel_leaf_size_);
  this->get_parameter("fitness_score_th", fitness_score_th_);
  this->get_parameter("vis_map_cloud_frame_interval", vis_map_cloud_frame_interval_);
  this->get_parameter("visualization.optimized_edge_width_m", optimized_edge_width_m_);
  this->get_parameter("visualization.original_edge_width_m", original_edge_width_m_);
  this->get_parameter("visualization.loop_edge_width_m", loop_edge_width_m_);
  this->get_parameter("expected_fixed_frame", input_contract_.fixed_frame);
  this->get_parameter("expected_body_frame", input_contract_.body_frame);
  this->get_parameter("require_exact_timestamps", input_contract_.require_exact_stamp);
  this->get_parameter("simple_stream_audit", stream_audit_enabled_);
  this->get_parameter("cloud_input_telemetry", cloud_input_telemetry_enabled_);

  if (!mapped_cloud_)
    throw std::invalid_argument(
        "mapped_cloud must be true for the global /cloud_registered_metric input");
  if (input_contract_.fixed_frame.empty() || input_contract_.body_frame.empty())
    throw std::invalid_argument("expected FAST-LIVO2 frames must not be empty");
  if (input_contract_.fixed_frame == input_contract_.body_frame)
    throw std::invalid_argument("expected FAST-LIVO2 fixed and body frames must differ");
  simple_loop_closure::validateMathSafetyParameters(
      keyframe_dist_th_, keyframe_angular_dist_th_, time_stamp_tolerance_,
      vis_map_cloud_frame_interval_);
  optimized_edge_width_m_ = simple_loop_closure::validateVisualizationLineWidth(
      "visualization.optimized_edge_width_m", optimized_edge_width_m_);
  original_edge_width_m_ = simple_loop_closure::validateVisualizationLineWidth(
      "visualization.original_edge_width_m", original_edge_width_m_);
  loop_edge_width_m_ = simple_loop_closure::validateVisualizationLineWidth(
      "visualization.loop_edge_width_m", loop_edge_width_m_);
  const std::uint32_t validated_sync_queue_size =
      simple_loop_closure::validateSyncQueueSize(sync_queue_size_);
  const std::uint32_t validated_cloud_input_depth =
      simple_loop_closure::validateCloudInputDepth(cloud_input_depth_);
  const std::uint32_t validated_pgo_output_depth =
      simple_loop_closure::validatePgoOutputDepth(pgo_output_depth_);
  if (loop_search_frame_interval_ < 0)
    throw std::invalid_argument("loop_search_frame_interval must not be negative");

  pub_pgo_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/pgo_odom", rclcpp::QoS(validated_pgo_output_depth));

  search_radius_ *= search_radius_;
  stop_loop_closure_thread_ = false;
  stop_visualize_thread_ = false;

  // This RMW receiver history is independent of the ExactTime tuple queue
  // constructed below: it controls samples waiting to reach this callback.
  rmw_qos_profile_t cloud_input_qos = rmw_qos_profile_default;
  cloud_input_qos.depth = validated_cloud_input_depth;
  cloud_input_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  cloud_input_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  if (cloud_input_telemetry_enabled_)
  {
    ingress_telemetry_ = std::make_shared<simple_loop_closure::IngressTelemetry>();
    const std::weak_ptr<simple_loop_closure::IngressTelemetry> weak_telemetry(ingress_telemetry_);
    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.event_callbacks.message_lost_callback =
        [weak_telemetry](rclcpp::QOSMessageLostInfo& info) noexcept {
          if (const auto telemetry = weak_telemetry.lock())
            telemetry->cloud_events.observeMessageLost(info.total_count, info.total_count_change);
        };
    cloud_options.event_callbacks.incompatible_qos_callback =
        [weak_telemetry](rclcpp::QOSRequestedIncompatibleQoSInfo& info) noexcept {
          if (const auto telemetry = weak_telemetry.lock())
          {
            telemetry->cloud_events.observeIncompatibleQos(
                info.total_count, info.total_count_change,
                static_cast<std::int32_t>(info.last_policy_kind));
          }
        };
    rclcpp::SubscriptionOptions odom_options;
    odom_options.event_callbacks.message_lost_callback =
        [weak_telemetry](rclcpp::QOSMessageLostInfo& info) noexcept {
          if (const auto telemetry = weak_telemetry.lock())
            telemetry->odom_events.observeMessageLost(info.total_count, info.total_count_change);
        };
    odom_options.event_callbacks.incompatible_qos_callback =
        [weak_telemetry](rclcpp::QOSRequestedIncompatibleQoSInfo& info) noexcept {
          if (const auto telemetry = weak_telemetry.lock())
          {
            telemetry->odom_events.observeIncompatibleQos(
                info.total_count, info.total_count_change,
                static_cast<std::int32_t>(info.last_policy_kind));
          }
        };

    try
    {
      sub_cloud_.subscribe(this, "/cloud", cloud_input_qos, cloud_options);
      ingress_telemetry_->cloud_events.setSupported(true);
    }
    catch (const rclcpp::UnsupportedEventTypeException& exception)
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Cloud subscription RMW event telemetry is unsupported (%s); continuing without event callbacks",
          exception.what());
      sub_cloud_.subscribe(this, "/cloud", cloud_input_qos);
    }
    try
    {
      sub_odom_.subscribe(this, "/odometry", rmw_qos_profile_default, odom_options);
      ingress_telemetry_->odom_events.setSupported(true);
    }
    catch (const rclcpp::UnsupportedEventTypeException& exception)
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Odometry subscription RMW event telemetry is unsupported (%s); continuing without event callbacks",
          exception.what());
      sub_odom_.subscribe(this, "/odometry");
    }

    cloud_telemetry_connection_ = sub_cloud_.registerCallback(
        [weak_telemetry](const sensor_msgs::msg::PointCloud2::ConstSharedPtr&) noexcept {
          if (const auto telemetry = weak_telemetry.lock())
            telemetry->observeCloudArrival(steadyNowNs());
        });
    odom_telemetry_connection_ = sub_odom_.registerCallback(
        [weak_telemetry](const nav_msgs::msg::Odometry::ConstSharedPtr&) noexcept {
          if (const auto telemetry = weak_telemetry.lock())
            telemetry->observeOdomArrival(steadyNowNs());
        });
  }
  else
  {
    sub_cloud_.subscribe(this, "/cloud", cloud_input_qos);
    sub_odom_.subscribe(this, "/odometry");
  }

  if (stream_audit_enabled_)
  {
    stream_audit_context_ = std::make_shared<StreamAuditContext>();
    const std::weak_ptr<StreamAuditContext> weak_context(stream_audit_context_);
    // These independent taps are registered before the synchronizer, so their
    // counts describe subscription ingress rather than synchronized output.
    cloud_audit_connection_ = sub_cloud_.registerCallback(
        [weak_context](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
          if (const auto context = weak_context.lock())
          {
            std::lock_guard<std::mutex> guard(context->mutex);
            context->audit.cloud_ingress.observe(rclcpp::Time(msg->header.stamp).nanoseconds());
          }
        });
    odom_audit_connection_ = sub_odom_.registerCallback(
        [weak_context](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
          if (const auto context = weak_context.lock())
          {
            std::lock_guard<std::mutex> guard(context->mutex);
            context->audit.odom_ingress.observe(rclcpp::Time(msg->header.stamp).nanoseconds());
          }
        });

    stream_audit_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/dump_stream_audit",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->message = makeStreamAuditReport("snapshot");
          response->success = !response->message.empty();
          if (response->success)
            std::cout << response->message << std::endl;
        });
  }

  synchronizer_ = std::make_shared<Sync>(
      SyncPolicy(validated_sync_queue_size), sub_cloud_, sub_odom_);
  synchronizer_->registerCallback(std::bind(&SimpleLoopClosureNode::pointCloudAndOdometryCallback, this,
                                            std::placeholders::_1, std::placeholders::_2));

  sub_save_req_ = this->create_subscription<std_msgs::msg::String>(
      "/save_req", 10, std::bind(&SimpleLoopClosureNode::saveRequestCallback, this, std::placeholders::_1));

  icp_.setMaximumIterations(50);
  icp_.setMaxCorrespondenceDistance(search_radius_ * 2.0);
  icp_.setTransformationEpsilon(0.0001);
  icp_.setEuclideanFitnessEpsilon(0.0001);
  icp_.setRANSACIterations(0);

  vg_target_.setLeafSize(target_voxel_leaf_size_, target_voxel_leaf_size_, target_voxel_leaf_size_);
  vg_source_.setLeafSize(source_voxel_leaf_size_, source_voxel_leaf_size_, source_voxel_leaf_size_);
  vg_map_.setLeafSize(vis_map_voxel_leaf_size_, vis_map_voxel_leaf_size_, vis_map_voxel_leaf_size_);

  loop_schedule_ = std::make_unique<simple_loop_closure::DeterministicLoopSchedule>(
      loop_search_frame_interval_);
  keyframes_cloud_.clear();
  keyframes_odom_.clear();

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(parameters);

  Eigen::VectorXd prior_noise_vector(6);
  prior_noise_vector << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
  prior_noise_ = gtsam::noiseModel::Diagonal::Variances(prior_noise_vector);
  Eigen::VectorXd odom_noise_vector(6);
  odom_noise_vector << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  odom_noise_ = gtsam::noiseModel::Diagonal::Variances(odom_noise_vector);

  odom_edge_color_.r = 1.0;
  odom_edge_color_.g = 0.0;
  odom_edge_color_.b = 1.0;
  odom_edge_color_.a = 1.0;

  loop_edge_color_.r = 1.0;
  loop_edge_color_.g = 0.0;
  loop_edge_color_.b = 0.0;
  loop_edge_color_.a = 1.0;

  node_color_.r = 0.5;
  node_color_.g = 1.0;
  node_color_.b = 0.0;
  node_color_.a = 1.0;

  node_scale_ = 0.15;

  saving_ = false;

  // Start Threads
  loop_close_thread_ = std::thread(&SimpleLoopClosureNode::loopCloseThread, this);
  visualize_thread_ = std::thread(&SimpleLoopClosureNode::visualizeThread, this);

  RCLCPP_INFO(this->get_logger(), "Loop Closure Node Initialized.");
}

simple_loop_closure::StreamAuditSnapshot SimpleLoopClosureNode::captureStreamAuditSnapshot()
{
  simple_loop_closure::StreamAuditSnapshot snapshot;
  std::scoped_lock guard(stream_audit_context_->mutex, mtx_buf_, mtx_res_);
  snapshot.audit = stream_audit_context_->audit;
  snapshot.cloud_subscriber_qos = formatQos(sub_cloud_.getSubscriber()->get_actual_qos());
  snapshot.pgo_publisher_qos = formatQos(pub_pgo_odometry_->get_actual_qos());
  snapshot.keyframes_stored = keyframes_odom_.size();
  snapshot.loop_factors = loop_edges_.size();
  return snapshot;
}

std::string SimpleLoopClosureNode::makeStreamAuditReport(const char* report_kind)
{
  if (!stream_audit_context_)
    return {};
  return simple_loop_closure::formatStreamAuditReport(captureStreamAuditSnapshot(), report_kind);
}

void SimpleLoopClosureNode::emitStreamAudit(const char* report_kind)
{
  const std::string report = makeStreamAuditReport(report_kind);
  if (!report.empty())
    std::cout << report << std::endl;
}

void SimpleLoopClosureNode::emitIngressTelemetry()
{
  if (!ingress_telemetry_)
    return;
  std::cout << simple_loop_closure::formatIngressTelemetryReport(
      ingress_telemetry_->snapshot(),
      formatQos(sub_cloud_.getSubscriber()->get_actual_qos()),
      formatQos(sub_odom_.getSubscriber()->get_actual_qos())) << std::endl;
}

bool SimpleLoopClosureNode::inputContractValid(
    const sensor_msgs::msg::PointCloud2& cloud_msg,
    const nav_msgs::msg::Odometry& odom_msg) const
{
  const auto& position = odom_msg.pose.pose.position;
  const auto& orientation = odom_msg.pose.pose.orientation;
  const bool frames_match = input_contract_.framesMatch(
      cloud_msg.header.frame_id, odom_msg.header.frame_id, odom_msg.child_frame_id);
  const bool stamps_match = input_contract_.stampsMatch(
      rclcpp::Time(cloud_msg.header.stamp).nanoseconds(),
      rclcpp::Time(odom_msg.header.stamp).nanoseconds());
  const bool pose_valid = input_contract_.poseIsValid(
      position.x, position.y, position.z,
      orientation.x, orientation.y, orientation.z, orientation.w);
  return frames_match && stamps_match && pose_valid && cloudSchemaValid(cloud_msg);
}

bool SimpleLoopClosureNode::cloudSchemaValid(
    const sensor_msgs::msg::PointCloud2& cloud_msg) const
{
  if (cloud_msg.width == 0 || cloud_msg.height == 0 || cloud_msg.data.empty())
    return false;

  const std::size_t expected_data_size =
      static_cast<std::size_t>(cloud_msg.row_step) * cloud_msg.height;
  return input_contract_.cloudFieldsMatch(
             cloud_msg.fields, sensor_msgs::msg::PointField::FLOAT32) &&
         expected_data_size <= cloud_msg.data.size();
}

void makeDirectory(const std::string& directory)
{
  if (!std::filesystem::is_directory(directory) || !std::filesystem::exists(directory))
  {                                                // Check if directory exists
    std::filesystem::create_directory(directory);  // create directory
  }
}

bool SimpleLoopClosureNode::saveEachFrames()
{
  std::string frames_directory = save_directory_ + "frames/";
  makeDirectory(frames_directory);

  int optimization_result_size = 0;
  {
    MtxLockGuard guard(mtx_res_);
    optimization_result_size = optimization_result_.size();
  }

  if (optimization_result_size <= 0)
    return false;

  int digits = std::to_string(optimization_result_size).length();
  if (digits < 6)
    digits = 6;

  std::string poses_csv_filename = save_directory_ + "poses.csv";
  std::ofstream poses_csv_file(poses_csv_filename);
  if (!poses_csv_file)
  {
    RCLCPP_WARN(this->get_logger(), "Failed to open poses CSV file: %s", poses_csv_filename.c_str());
    return false;
  }
  poses_csv_file << "index, timestamp, x, y, z, qx, qy, qz, qw" << std::endl;
  for (int i = 0; i < optimization_result_size; i++)
  {
    Eigen::Affine3d pose;
    {
      MtxLockGuard guard(mtx_res_);
      pose = Eigen::Affine3d(optimization_result_.at<gtsam::Pose3>(i).matrix());
    }

    PointCloudType copied_cloud;
    {
      MtxLockGuard guard(mtx_buf_);
      copied_cloud = *keyframes_cloud_[i];
    }

    std::stringstream frame_filename_str;
    frame_filename_str << std::setfill('0') << std::right << std::setw(digits) << i << ".pcd";
    Eigen::Quaterniond quat(pose.rotation());
    Eigen::Vector3d trans = pose.translation();
    pcl::io::savePCDFileBinary(frames_directory + frame_filename_str.str(), copied_cloud);
    poses_csv_file << std::fixed << i << ", " << pcl_conversions::fromPCL(copied_cloud.header.stamp).seconds() << ", "
                   << trans.x() << ", " << trans.y() << ", " << trans.z() << ", " << quat.x() << ", " << quat.y()
                   << ", " << quat.z() << ", " << quat.w() << std::endl;
  }

  poses_csv_file.close();

  return true;
}

void SimpleLoopClosureNode::saveThread()
{
  PointCloudType::Ptr map_cloud = constructPointCloudMap();

  if (map_cloud == nullptr || map_cloud->empty())
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Point cloud map is empty.");
  }
  else
  {
    try
    {
      if (!saveEachFrames())
      {
        throw std::runtime_error("Save failed");
      }
      std::cout<<"SAVE DIRECTORY: "<<save_directory_<<std::endl;
      pcl::io::savePCDFileBinary(save_directory_ + "map.pcd", *map_cloud);
      RCLCPP_INFO_STREAM(this->get_logger(), "Save completed.");
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(this->get_logger(), "Save failed: %s", e.what());
    }
  }
  saving_ = false;
}

void SimpleLoopClosureNode::saveRequestCallback(const std_msgs::msg::String::ConstSharedPtr& directory)
{
  if (saving_ == true)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Already in process. Request is denied.");
    return;
  }

  save_directory_ = directory->data;
  if (!save_directory_.empty() && save_directory_.back() != '/')
    save_directory_ += '/';

  RCLCPP_INFO_STREAM(this->get_logger(), "Start Saving.");
  if (save_thread_.joinable())
    save_thread_.join();

  saving_ = true;
  save_thread_ = std::thread(&SimpleLoopClosureNode::saveThread, this);
}

void SimpleLoopClosureNode::publishPoseGraphOptimizedOdometry(const Eigen::Affine3d& affine_curr,
                                                              const nav_msgs::msg::Odometry& odom_msg)
{
  Eigen::Affine3d pgo_affine;

  Eigen::Affine3d optimized_pose_last;
  int optimized_pose_id_last;
  getLastOptimizedPose(optimized_pose_last, optimized_pose_id_last);

  if (optimized_pose_id_last != 0)
  {
    MtxLockGuard guard(mtx_buf_);
    pgo_affine = optimized_pose_last * (keyframes_odom_[optimized_pose_id_last]->inverse() * affine_curr);
  }
  else
  {
    pgo_affine = affine_curr;
  }

  nav_msgs::msg::Odometry pgo_odom_msg = odom_msg;
  pgo_odom_msg.pose.pose = tf2::toMsg(pgo_affine);
  if (!stream_audit_context_)
  {
    pub_pgo_odometry_->publish(pgo_odom_msg);
    return;
  }

  const std::int64_t stamp_ns = rclcpp::Time(pgo_odom_msg.header.stamp).nanoseconds();
  std::lock_guard<std::mutex> guard(stream_audit_context_->mutex);
  stream_audit_context_->audit.pgo_publish_attempted.observe(stamp_ns);
  try
  {
    pub_pgo_odometry_->publish(pgo_odom_msg);
  }
  catch (...)
  {
    stream_audit_context_->audit.pgo_publish_failed.observe(stamp_ns);
    throw;
  }
  stream_audit_context_->audit.pgo_publish_completed.observe(stamp_ns);
}

void SimpleLoopClosureNode::pointCloudAndOdometryCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
{
  ScopedSyncServiceTimer telemetry_timer(ingress_telemetry_);
  const std::int64_t cloud_stamp_ns = rclcpp::Time(cloud_msg->header.stamp).nanoseconds();
  const std::int64_t odom_stamp_ns = rclcpp::Time(odom_msg->header.stamp).nanoseconds();
  if (stream_audit_context_)
  {
    std::lock_guard<std::mutex> guard(stream_audit_context_->mutex);
    stream_audit_context_->audit.sync_callbacks.observe(cloud_stamp_ns, odom_stamp_ns);
  }

  if (!inputContractValid(*cloud_msg, *odom_msg))
  {
    if (stream_audit_context_)
    {
      std::lock_guard<std::mutex> guard(stream_audit_context_->mutex);
      stream_audit_context_->audit.contract_rejected.observe(cloud_stamp_ns, odom_stamp_ns);
    }
    RCLCPP_ERROR(
        this->get_logger(),
        "Rejecting FAST-LIVO2 pair: cloud frame='%s', odom frame='%s', child='%s', cloud stamp=%ld, odom stamp=%ld",
        cloud_msg->header.frame_id.c_str(), odom_msg->header.frame_id.c_str(), odom_msg->child_frame_id.c_str(),
        rclcpp::Time(cloud_msg->header.stamp).nanoseconds(), rclcpp::Time(odom_msg->header.stamp).nanoseconds());
    return;
  }

  if (stream_audit_context_)
  {
    std::lock_guard<std::mutex> guard(stream_audit_context_->mutex);
    stream_audit_context_->audit.contract_accepted.observe(cloud_stamp_ns, odom_stamp_ns);
  }

  Affine3dPtr affine_curr(new Eigen::Affine3d);
  tf2::fromMsg(odom_msg->pose.pose, *affine_curr);

  publishPoseGraphOptimizedOdometry(*affine_curr, *odom_msg);

  simple_loop_closure::KeyframeDecision decision;
  {
    MtxLockGuard guard(mtx_buf_);
    const Eigen::Affine3d* previous_pose =
        keyframes_odom_.empty() ? nullptr : keyframes_odom_.back().get();
    decision = simple_loop_closure::decideKeyframe(
        previous_pose, *affine_curr, keyframe_dist_th_, keyframe_angular_dist_th_);
  }

  if (stream_audit_context_)
  {
    std::lock_guard<std::mutex> guard(stream_audit_context_->mutex);
    if (decision.is_keyframe)
      stream_audit_context_->audit.keyframe_selected.observe(odom_stamp_ns);
    else
      stream_audit_context_->audit.keyframe_skipped.observe(odom_stamp_ns);
  }
  if (!decision.is_keyframe)
    return;

  PointCloudType::Ptr cloud_curr(new PointCloudType);
  pcl::fromROSMsg(*cloud_msg, *cloud_curr);
  if (stream_audit_context_)
  {
    std::lock_guard<std::mutex> guard(stream_audit_context_->mutex);
    stream_audit_context_->audit.pcl_converted.observe(cloud_stamp_ns, odom_stamp_ns);
  }

  if (mapped_cloud_)
  {
    PointCloudType::Ptr cloud_base(new PointCloudType);
    Eigen::Affine3d affine_inv = affine_curr->inverse();
    pcl::transformPointCloud(*cloud_curr, *cloud_base, affine_inv);
    cloud_base->header.frame_id = input_contract_.body_frame;
    cloud_curr = cloud_base;
  }

  {
    MtxLockGuard guard(mtx_buf_);
    const bool first_keyframe = keyframes_odom_.empty();
    if (first_keyframe)
      odom_frame_id_ = odom_msg->header.frame_id;
    trajectory_dist_.push_back(simple_loop_closure::nextTrajectoryDistance(
        first_keyframe, first_keyframe ? 0.0 : trajectory_dist_.back(), decision));

    keyframes_cloud_.push_back(cloud_curr);
    keyframes_odom_.push_back(affine_curr);
    latest_input_header_ = odom_msg->header;
  }
  if (stream_audit_context_)
  {
    std::lock_guard<std::mutex> guard(stream_audit_context_->mutex);
    stream_audit_context_->audit.keyframes_inserted.observe(odom_stamp_ns);
  }
  keyframe_changed_.notify_one();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SimpleLoopClosureNode::constructPointCloudMap(const int interval)
{
  int optimization_result_size = 0;
  int first_cloud_size = 0;
  {
    MtxLockGuard guard(mtx_res_);
    optimization_result_size = optimization_result_.size();
  }
  {
    MtxLockGuard guard(mtx_buf_);
    if (optimization_result_size > 0 && !keyframes_cloud_.empty())
      first_cloud_size = keyframes_cloud_.front()->size();
  }

  if (optimization_result_size <= 0)
    return nullptr;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new PointCloudType);
  map_cloud->reserve((size_t)((double)(first_cloud_size * optimization_result_size) * 1.5));
  const std::size_t stride = static_cast<std::size_t>(interval) + 1U;
  for (std::size_t i = 0; i < static_cast<std::size_t>(optimization_result_size); i += stride)
  {
    Eigen::Affine3d pose;
    {
      MtxLockGuard guard(mtx_res_);
      pose = Eigen::Affine3d(optimization_result_.at<gtsam::Pose3>(i).matrix());
    }

    PointCloudType transformed_cloud;
    {
      MtxLockGuard guard(mtx_buf_);
      pcl::transformPointCloud(*keyframes_cloud_[i], transformed_cloud, pose);
    }

    *map_cloud += transformed_cloud;
  }

  return map_cloud;
}

void SimpleLoopClosureNode::publishMapCloud(const PointCloudType::Ptr& map_cloud)
{
  if (map_cloud == NULL)
    return;

  PointCloudType map_cloud_ds_;

  vg_map_.setInputCloud(map_cloud);
  vg_map_.filter(map_cloud_ds_);

  sensor_msgs::msg::PointCloud2 map_cloud_msg;
  pcl::toROSMsg(map_cloud_ds_, map_cloud_msg);
  {
    MtxLockGuard guard(mtx_buf_);
    map_cloud_msg.header = latest_input_header_;
  }

  if (map_cloud_msg.header.frame_id != input_contract_.fixed_frame)
    return;

  pub_map_cloud_->publish(map_cloud_msg);
}

void SimpleLoopClosureNode::constructVisualizationOdometryEdges(const int& res_size,
                                                                const std_msgs::msg::Header& header,
                                                                visualization_msgs::msg::Marker& marker_msg)
{
  marker_msg.header = header;
  marker_msg.ns = "optimized_keyframes";
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.scale.x = optimized_edge_width_m_;
  marker_msg.color = odom_edge_color_;

  marker_msg.points.clear();
  marker_msg.points.reserve(res_size * 2);
  for (int i = 0; i < res_size - 1; i++)
  {
    MtxLockGuard guard(mtx_res_);
    gtsam::Pose3 pose1 = optimization_result_.at<gtsam::Pose3>(i);
    gtsam::Pose3 pose2 = optimization_result_.at<gtsam::Pose3>(i + 1);

    geometry_msgs::msg::Point p1, p2;
    p1.x = pose1.x();
    p1.y = pose1.y();
    p1.z = pose1.z();
    p2.x = pose2.x();
    p2.y = pose2.y();
    p2.z = pose2.z();
    marker_msg.points.emplace_back(p1);
    marker_msg.points.emplace_back(p2);
  }
}

void SimpleLoopClosureNode::constructVisualizationOriginalEdges(
    const int& res_size, const std_msgs::msg::Header& header,
    visualization_msgs::msg::Marker& marker_msg)
{
  marker_msg.header = header;
  marker_msg.ns = "original_keyframes";
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_msg.scale.x = original_edge_width_m_;
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 1.0;
  marker_msg.color.a = 1.0;

  marker_msg.points.clear();
  marker_msg.points.reserve(res_size);
  MtxLockGuard guard(mtx_buf_);
  const int original_size = std::min(res_size, static_cast<int>(keyframes_odom_copied_.size()));
  for (int i = 0; i < original_size; ++i)
  {
    geometry_msgs::msg::Point point;
    point.x = keyframes_odom_copied_[i]->translation().x();
    point.y = keyframes_odom_copied_[i]->translation().y();
    point.z = keyframes_odom_copied_[i]->translation().z();
    marker_msg.points.emplace_back(point);
  }
}

void SimpleLoopClosureNode::constructVisualizationLoopEdges(const int& res_size, const std_msgs::msg::Header& header,
                                                            visualization_msgs::msg::Marker& marker_msg)
{
  int edge_num;
  {
    MtxLockGuard guard(mtx_res_);
    edge_num = loop_edges_.size();
  }

  marker_msg.header = header;
  marker_msg.ns = "loop_edges";
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.scale.x = loop_edge_width_m_;
  marker_msg.color = loop_edge_color_;

  marker_msg.points.clear();
  marker_msg.points.reserve(edge_num * 2);
  for (int i = 0; i < edge_num; i++)
  {
    MtxLockGuard guard(mtx_res_);

    int id1 = loop_edges_[i].first;
    int id2 = loop_edges_[i].second;

    if (id1 >= res_size || id2 >= res_size)
      continue;

    gtsam::Pose3 pose1 = optimization_result_.at<gtsam::Pose3>(id1);
    gtsam::Pose3 pose2 = optimization_result_.at<gtsam::Pose3>(id2);

    geometry_msgs::msg::Point p1, p2;
    p1.x = pose1.x();
    p1.y = pose1.y();
    p1.z = pose1.z();
    p2.x = pose2.x();
    p2.y = pose2.y();
    p2.z = pose2.z();
    marker_msg.points.emplace_back(p1);
    marker_msg.points.emplace_back(p2);
  }
}

void SimpleLoopClosureNode::constructVisualizationNodes(const int& res_size, const std_msgs::msg::Header& header,
                                                        visualization_msgs::msg::Marker& marker_msg)
{
  marker_msg.header = header;
  marker_msg.ns = "nodes";
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker_msg.scale.x = node_scale_;
  marker_msg.scale.y = node_scale_;
  marker_msg.scale.z = node_scale_;
  marker_msg.color = node_color_;

  marker_msg.points.clear();
  marker_msg.points.reserve(res_size);
  for (int i = 0; i < res_size; i++)
  {
    MtxLockGuard guard(mtx_res_);
    gtsam::Pose3 pose1 = optimization_result_.at<gtsam::Pose3>(i);

    geometry_msgs::msg::Point p1;
    p1.x = pose1.x();
    p1.y = pose1.y();
    p1.z = pose1.z();
    marker_msg.points.emplace_back(p1);
  }
}

void SimpleLoopClosureNode::publishVisualizationGraph()
{
  int optimization_result_size;
  {
    MtxLockGuard guard(mtx_res_);
    optimization_result_size = optimization_result_.size();
  }

  if (optimization_result_size <= 0)
    return;

  std_msgs::msg::Header header;
  {
    MtxLockGuard guard(mtx_buf_);
    header = latest_input_header_;
  }
  if (header.frame_id != input_contract_.fixed_frame)
    return;

  visualization_msgs::msg::MarkerArray marker_array_msg;
  marker_array_msg.markers.resize(4);
  constructVisualizationOriginalEdges(optimization_result_size, header, marker_array_msg.markers[0]);
  constructVisualizationOdometryEdges(optimization_result_size, header, marker_array_msg.markers[1]);
  constructVisualizationLoopEdges(optimization_result_size, header, marker_array_msg.markers[2]);
  constructVisualizationNodes(optimization_result_size, header, marker_array_msg.markers[3]);

  pub_vis_pose_graph_->publish(marker_array_msg);
  pub_original_keyframes_->publish(marker_array_msg.markers[0]);
  pub_optimized_keyframes_->publish(marker_array_msg.markers[1]);
  pub_loop_edges_->publish(marker_array_msg.markers[2]);
}

void SimpleLoopClosureNode::copyKeyFrames()
{
  MtxLockGuard guard(mtx_buf_);
  for (size_t i = keyframes_cloud_copied_.size(); i < keyframes_cloud_.size(); i++)
  {
    keyframes_cloud_copied_.push_back(keyframes_cloud_[i]);
    keyframes_odom_copied_.push_back(keyframes_odom_[i]);
    trajectory_dist_copied_.push_back(trajectory_dist_[i]);
  }
}

void SimpleLoopClosureNode::getLastOptimizedPose(Eigen::Affine3d& pose, int& id)
{
  MtxLockGuard guard(mtx_res_);
  if (!optimization_result_.empty())
  {
    pose = optimization_result_.at<gtsam::Pose3>(optimization_result_.size() - 1).matrix();
    id = optimization_result_.size() - 1;
  }
  else
  {
    MtxLockGuard guard(mtx_buf_);
    if (!keyframes_odom_.empty())
    {
      pose = *keyframes_odom_[0];
      id = 0;
    }
    else
    {
      pose = Eigen::Affine3d();
      id = 0;
    }
  }
}

bool SimpleLoopClosureNode::constructNextOdometryGraph(gtsam::NonlinearFactorGraph& graph,
                                                       gtsam::Values& init_estimate)
{
  const std::size_t id = loop_schedule_->nextOdometryId();
  if (id >= keyframes_odom_copied_.size())
    return false;

  if (id == 0U)
  {
    gtsam::Pose3 pose = gtsam::Pose3(keyframes_odom_copied_[0]->matrix());
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose, prior_noise_));
    init_estimate.insert(0, pose);
    return true;
  }

  Eigen::Affine3d optimized_pose_previous;
  {
    MtxLockGuard guard(mtx_res_);
    if (optimization_result_.size() != id)
      throw std::logic_error("optimizer/keyframe scheduling parity was lost");
    optimized_pose_previous =
        Eigen::Affine3d(optimization_result_.at<gtsam::Pose3>(id - 1U).matrix());
  }

  const Eigen::Affine3d pose_diff =
      keyframes_odom_copied_[id - 1U]->inverse() * (*keyframes_odom_copied_[id]);
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      id - 1U, id, gtsam::Pose3(pose_diff.matrix()), odom_noise_));
  const Eigen::Affine3d pose_init = optimized_pose_previous * pose_diff;
  init_estimate.insert(id, gtsam::Pose3(pose_init.matrix()));
  return true;
}

bool SimpleLoopClosureNode::snapshotOptimizedPrefix(
    const std::size_t prefix_size, std::vector<Eigen::Affine3d>& poses)
{
  MtxLockGuard guard(mtx_res_);
  if (prefix_size == 0U || optimization_result_.size() < prefix_size)
    return false;
  poses.clear();
  poses.reserve(prefix_size);
  for (std::size_t i = 0; i < prefix_size; ++i)
    poses.emplace_back(optimization_result_.at<gtsam::Pose3>(i).matrix());
  return true;
}

bool SimpleLoopClosureNode::constructKDTreeMatrix(
    const std::vector<Eigen::Affine3d>& poses, KDTreeMatrix& kdtree_mat)
{
  if (poses.empty())
    return false;

  kdtree_mat.resize(poses.size(), 3);
  for (std::size_t i = 0; i < poses.size(); ++i)
    kdtree_mat.row(i) = poses[i].translation();
  return true;
}

int SimpleLoopClosureNode::searchTarget(const KDTree& kdtree, const int& id_query, const Eigen::Affine3d& pose_query,
                                        const rclcpp::Time& stamp_query,
                                        const std::vector<Eigen::Affine3d>& poses)
{
  NanoFlannSearchResult search_result;
  search_result.reserve(1000);
#if NANOFLANN_VERSION >= 0x150
  nanoflann::SearchParameters search_params(0.0, true);
  kdtree.index_->radiusSearch(pose_query.translation().data(), search_radius_, search_result, search_params);
#else
  nanoflann::SearchParams search_params(0, 0.0, true);
  kdtree.index->radiusSearch(pose_query.translation().data(), search_radius_, search_result, search_params);
#endif
  std::sort(search_result.begin(), search_result.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.second != rhs.second)
      return lhs.second < rhs.second;
    return lhs.first < rhs.first;
  });

  Eigen::Quaterniond quat_query(pose_query.rotation());

  int target_id = -1;
  for (size_t i = 0; i < search_result.size(); i++)
  {
    int tmp_id = search_result[i].first;
    if (tmp_id < 0 || static_cast<std::size_t>(tmp_id) >= poses.size())
      continue;
    rclcpp::Time tmp_stamp = pcl_conversions::fromPCL(keyframes_cloud_copied_[tmp_id]->header.stamp);
    double time_diff = std::fabs(stamp_query.seconds() - tmp_stamp.seconds());

    Eigen::Quaterniond quat_target(poses[tmp_id].rotation());
    double angle_diff = quat_query.angularDistance(quat_target);
    double dist_diff = std::fabs(trajectory_dist_copied_[id_query] - trajectory_dist_copied_[tmp_id]);
    if (time_diff > loop_search_time_diff_th_ && angle_diff < loop_search_angular_dist_th_ &&
        dist_diff > loop_search_dist_diff_th_)
    {
      target_id = tmp_id;
      break;
    }
  }

  return target_id;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SimpleLoopClosureNode::constructTargetCloud(
    const int target_id, const std::vector<Eigen::Affine3d>& poses)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new PointCloudType);
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_ds(new PointCloudType);

  const int first_id = std::max(0, target_id - target_frame_num_);
  const int last_id = std::min(static_cast<int>(poses.size()) - 1, target_id + target_frame_num_);
  for (int i = first_id; i <= last_id; ++i)
  {
    PointCloudType tmp_cloud;
    pcl::transformPointCloud(*keyframes_cloud_copied_[i], tmp_cloud, poses[i]);

    *target_cloud += tmp_cloud;
  }

  if (target_voxel_leaf_size_ <= 0.0)
  {
    target_cloud_ds = target_cloud;
  }
  else
  {
    vg_target_.setInputCloud(target_cloud);
    vg_target_.filter(*target_cloud_ds);
  }

  return target_cloud_ds;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SimpleLoopClosureNode::constructSourceCloud(const int source_id)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_ds(new PointCloudType);

  if (source_voxel_leaf_size_ <= 0.0)
  {
    *source_cloud_ds = *keyframes_cloud_copied_[source_id];
  }
  else
  {
    vg_source_.setInputCloud(keyframes_cloud_copied_[source_id]);
    vg_source_.filter(*source_cloud_ds);
  }

  return source_cloud_ds;
}

bool SimpleLoopClosureNode::tryRegistration(const Eigen::Affine3d& init_pose, const PointCloudType::Ptr& source_cloud,
                                            const PointCloudType::Ptr& target_cloud, Eigen::Affine3d& result,
                                            double& score)
{
  PointCloudType::Ptr unused_cloud(new PointCloudType);
  icp_.setInputSource(source_cloud);
  icp_.setInputTarget(target_cloud);
  icp_.align(*unused_cloud, init_pose.matrix().cast<float>());

  Eigen::Affine3f result_f(icp_.getFinalTransformation());
  result = result_f.cast<double>();
  score = icp_.getFitnessScore();

  if (icp_.hasConverged() && score < fitness_score_th_)
    return true;

  return false;
}

bool SimpleLoopClosureNode::constructLoopEdge(const std::size_t source_id,
                                              gtsam::NonlinearFactorGraph& graph,
                                              LoopEdgeID& accepted_edge)
{
  if (source_id >= keyframes_cloud_copied_.size())
    return false;

  std::vector<Eigen::Affine3d> poses;
  if (!snapshotOptimizedPrefix(source_id + 1U, poses))
    throw std::logic_error("candidate pose prefix is not optimized");

  KDTreeMatrix kdtree_mat;
  if (!constructKDTreeMatrix(poses, kdtree_mat))
    return false;

  KDTree kdtree(3, std::cref(kdtree_mat), 10);
#if NANOFLANN_VERSION >= 0x150
  kdtree.index_->buildIndex();
#else
  kdtree.index->buildIndex();
#endif

  const int source_id_int = static_cast<int>(source_id);
  const Eigen::Affine3d& pose_query = poses[source_id];
  rclcpp::Time stamp_query = pcl_conversions::fromPCL(keyframes_cloud_copied_[source_id]->header.stamp);

  int target_id = searchTarget(kdtree, source_id_int, pose_query, stamp_query, poses);

  if (target_id == -1)
    return false;

  PointCloudType::Ptr target_cloud = constructTargetCloud(target_id, poses);
  PointCloudType::Ptr source_cloud = constructSourceCloud(source_id_int);
  Eigen::Affine3d registration_result;
  double fitness_score;
  if (!tryRegistration(pose_query, source_cloud, target_cloud, registration_result, fitness_score))
    return false;

  Eigen::VectorXd noise_vector(6);
  noise_vector << fitness_score, fitness_score, fitness_score, fitness_score, fitness_score, fitness_score;
  gtsam::SharedNoiseModel constraint_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1), gtsam::noiseModel::Diagonal::Variances(noise_vector));

  Eigen::Affine3d pose_diff = registration_result.inverse() * poses[target_id];
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      source_id, target_id, gtsam::Pose3(pose_diff.matrix()), constraint_noise));
  accepted_edge = LoopEdgeID(source_id_int, target_id);
  return true;
}

bool SimpleLoopClosureNode::updateISAM2(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& init_estimate)
{
  if (graph.empty())
    return false;

  if (init_estimate.empty())
    isam2_.update(graph);
  else
    isam2_.update(graph, init_estimate);
  isam2_.update();

  {
    MtxLockGuard guard(mtx_res_);
    optimization_result_ = isam2_.calculateEstimate();
  }

  return true;
}

void SimpleLoopClosureNode::processAvailableKeyFrames()
{
  while (loop_schedule_->nextOdometryId() < keyframes_odom_copied_.size())
  {
    const std::size_t odometry_id = loop_schedule_->nextOdometryId();
    gtsam::NonlinearFactorGraph odometry_graph;
    gtsam::Values init_estimate;
    if (!constructNextOdometryGraph(odometry_graph, init_estimate) ||
        !updateISAM2(odometry_graph, init_estimate))
      throw std::logic_error("failed to add the next deterministic odometry node");
    loop_schedule_->markOdometryAdded(odometry_id);

    if (!loop_schedule_->candidateReady())
      continue;

    const std::size_t source_id = loop_schedule_->nextCandidateId();
    gtsam::NonlinearFactorGraph loop_graph;
    LoopEdgeID accepted_edge(-1, -1);
    const bool accepted = constructLoopEdge(source_id, loop_graph, accepted_edge);
    loop_schedule_->markCandidateSearched(source_id);
    if (!accepted)
      continue;

    gtsam::Values no_initial_estimate;
    if (!updateISAM2(loop_graph, no_initial_estimate))
      throw std::logic_error("failed to apply an accepted deterministic loop edge");
    {
      MtxLockGuard guard(mtx_res_);
      loop_edges_.push_back(accepted_edge);
    }
    loop_schedule_->recordAcceptedEdge(
        static_cast<std::size_t>(accepted_edge.first), static_cast<std::size_t>(accepted_edge.second));
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Loop Detected:" << accepted_edge.first << " -> " << accepted_edge.second);
  }
}

void SimpleLoopClosureNode::publishDeterminismAudit()
{
  std::size_t produced_count;
  std::size_t copied_count;
  {
    MtxLockGuard guard(mtx_buf_);
    produced_count = keyframes_cloud_.size();
    copied_count = keyframes_cloud_copied_.size();
  }

  std::size_t graph_node_count;
  std::size_t accepted_pair_count;
  {
    MtxLockGuard guard(mtx_res_);
    graph_node_count = optimization_result_.size();
    accepted_pair_count = loop_edges_.size();
  }
  const std::size_t graph_factor_count = isam2_.getFactorsUnsafe().size();
  const std::size_t added_count = loop_schedule_->addedCount();
  const std::size_t searched_count = loop_schedule_->searchedCount();
  const std::size_t expected_searched_count = loop_schedule_->expectedSearchedCount(produced_count);
  const std::size_t expected_factor_count = added_count + accepted_pair_count;
  const bool parity = !loop_worker_failed_ && produced_count == copied_count &&
                      copied_count == added_count && added_count == graph_node_count &&
                      searched_count == expected_searched_count &&
                      loop_schedule_->searchedPrefixMatches(produced_count) &&
                      accepted_pair_count == loop_schedule_->acceptedCount() &&
                      graph_factor_count == expected_factor_count;

  const char* status = parity ? "PASS" : "FAIL";
  if (parity)
  {
    RCLCPP_INFO(
        this->get_logger(),
        "SLC_DETERMINISM_AUDIT status=%s produced=%zu copied=%zu added=%zu searched=%zu "
        "expected_searched=%zu accepted=%zu graph_nodes=%zu graph_factors=%zu "
        "searched_digest=%s accepted_digest=%s",
        status, produced_count, copied_count, added_count, searched_count, expected_searched_count,
        accepted_pair_count, graph_node_count, graph_factor_count,
        loop_schedule_->searchedDigest().c_str(), loop_schedule_->acceptedDigest().c_str());
  }
  else
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "SLC_DETERMINISM_AUDIT status=%s produced=%zu copied=%zu added=%zu searched=%zu "
        "expected_searched=%zu accepted=%zu graph_nodes=%zu graph_factors=%zu expected_factors=%zu "
        "searched_digest=%s accepted_digest=%s worker_failed=%d",
        status, produced_count, copied_count, added_count, searched_count, expected_searched_count,
        accepted_pair_count, graph_node_count, graph_factor_count, expected_factor_count,
        loop_schedule_->searchedDigest().c_str(), loop_schedule_->acceptedDigest().c_str(),
        loop_worker_failed_.load() ? 1 : 0);
  }
}

void SimpleLoopClosureNode::shutdownAndDrain()
{
  bool expected = false;
  if (!shutdown_started_.compare_exchange_strong(expected, true))
    return;

  stop_visualize_thread_ = true;
  stop_loop_closure_thread_ = true;
  keyframe_changed_.notify_all();

  if (loop_close_thread_.joinable())
    loop_close_thread_.join();
  if (visualize_thread_.joinable())
    visualize_thread_.join();

  // The final optimizer update is complete before this last native graph snapshot.
  try
  {
    publishVisualizationGraph();
    if (!pub_vis_pose_graph_->wait_for_all_acked(std::chrono::seconds(1)))
      RCLCPP_WARN(this->get_logger(), "Final graph snapshot was published but not acknowledged within 1 second");
  }
  catch (const std::exception& exception)
  {
    RCLCPP_WARN(this->get_logger(), "Unable to publish the final graph snapshot: %s", exception.what());
  }
  publishDeterminismAudit();
  cloud_audit_connection_.disconnect();
  odom_audit_connection_.disconnect();
  cloud_telemetry_connection_.disconnect();
  odom_telemetry_connection_.disconnect();
  emitStreamAudit("shutdown");
  emitIngressTelemetry();
}

void SimpleLoopClosureNode::visualizeThread()
{
  rclcpp::Rate rate(1);  // 1 Hz
  while (rclcpp::ok() && !stop_visualize_thread_)
  {
    PointCloudType::Ptr map_cloud = constructPointCloudMap(vis_map_cloud_frame_interval_);
    publishMapCloud(map_cloud);
    publishVisualizationGraph();
    rate.sleep();
  }
}

void SimpleLoopClosureNode::loopCloseThread()
{
  try
  {
    for (;;)
    {
      copyKeyFrames();
      processAvailableKeyFrames();

      std::unique_lock<std::mutex> lock(mtx_buf_);
      const bool drained = keyframes_cloud_copied_.size() == keyframes_cloud_.size() &&
                           loop_schedule_->addedCount() == keyframes_cloud_copied_.size();
      if (stop_loop_closure_thread_ && drained)
        break;
      keyframe_changed_.wait(lock, [&]() {
        return stop_loop_closure_thread_ || keyframes_cloud_.size() > keyframes_cloud_copied_.size();
      });
    }
  }
  catch (const std::exception& exception)
  {
    loop_worker_failed_ = true;
    RCLCPP_ERROR(this->get_logger(), "Deterministic loop worker failed: %s", exception.what());
  }
}

void SimpleLoopClosureNode::spin()
{
  RCLCPP_INFO(this->get_logger(), "Loop Closure Started");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(shared_from_this());
  while (rclcpp::ok() && shutdown_signal_received == 0)
    executor.spin_once(std::chrono::milliseconds(100));
  shutdownAndDrain();
  executor.remove_node(shared_from_this());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestStagedShutdown);
  std::signal(SIGTERM, requestStagedShutdown);
  auto loop_closure_node = std::make_shared<SimpleLoopClosureNode>();
  loop_closure_node->spin();
  rclcpp::shutdown();
  return 0;
}
