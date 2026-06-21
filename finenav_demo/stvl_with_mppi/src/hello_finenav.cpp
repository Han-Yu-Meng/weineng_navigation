
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_set>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// FineNav core
#include "finenav_engine/finenav_engine.hpp"
#include "grid_map.hpp" // TODO: resolve anti-pattern export
#include "astar_path_search.hpp"

#include "occ_grid_map_view.hpp"
#include "occ_grid_2d_map.hpp"
#include "finenav_mppi_controller/controller.hpp"
#include "finenav_util/cloud_publish_helper.hpp"
#include "seek_to_nearest_point.hpp"
#include "trim_by_aabb.hpp"
#include "trim_by_distance.hpp"
#include "smooth_path.hpp"

// ROS messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// Local demo modules
#include "stvl_manager.hpp"
#include "terrain_mapview.hpp"
#include "finenav_util/cloud_publish_helper.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trim_by_distance.hpp"
#include "seek_to_nearest_point.hpp"
#include "trim_by_aabb.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace finenav;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", false); // Use time in /clock // TODO: xxx
    auto node = rclcpp::Node::make_shared("hello_finenav", options); // TODO: inhirent options?

    auto finenav_engine = std::make_shared<FineNavEngine>(node);

    auto localizer = finenav_engine->getLocalizer();

    auto stvl_manager = std::make_shared<StvlManager>(node);

    // ── Covariance parameters (all loaded from obs_cov.yaml) ──────────────
    //
    //  obs_cov.global_pose_cov_* : covariance for /localization_pose feed.
    //    Trust x/y/z/yaw (small), DO NOT trust roll/pitch (large) because
    //    the global pose source has no reliable absolute roll/pitch reference.
    //
    //  obs_cov.odom_pose_cov_*   : covariance for /Odometry → pose channel.
    //    ONLY trust roll/pitch (small); x/y/z/yaw are set to large values
    //    so only the relative-pose channel constrains those DOFs.
    //
    //  obs_cov.odom_rel_cov_*    : covariance for /Odometry → relativePose channel.
    //    All 6 DOFs set to the expected inter-frame increment uncertainty.
    //
    //  All parameters are re-read on every callback to support live tuning.

    // ------------------------------ Launch Localization ----------------------------------

    // ── 从 obs_cov.yaml 读取观测协方差 ────────────────────────────────────────
    // Global pose (/localization_pose): 不信任 roll / pitch
    node->declare_parameter("obs_cov.global_pose_cov_x",     0.01);
    node->declare_parameter("obs_cov.global_pose_cov_y",     0.01);
    node->declare_parameter("obs_cov.global_pose_cov_z",     0.1);
    node->declare_parameter("obs_cov.global_pose_cov_roll",  1.0e9);  // 不信任
    node->declare_parameter("obs_cov.global_pose_cov_pitch", 1.0e9);  // 不信任
    node->declare_parameter("obs_cov.global_pose_cov_yaw",   0.001);

    // Odometry abs pose (/Odometry → pose channel): 仅信任 roll / pitch
    node->declare_parameter("obs_cov.odom_pose_cov_x",     1.0e6);   // 不信任
    node->declare_parameter("obs_cov.odom_pose_cov_y",     1.0e6);   // 不信任
    node->declare_parameter("obs_cov.odom_pose_cov_z",     1.0e6);   // 不信任
    node->declare_parameter("obs_cov.odom_pose_cov_roll",  0.00001); // 信任 (IMU)
    node->declare_parameter("obs_cov.odom_pose_cov_pitch", 0.00001); // 信任 (IMU)
    node->declare_parameter("obs_cov.odom_pose_cov_yaw",   1.0e6);   // 不信任

    // Odometry twist (/Odometry → twist channel): 直接注入角速度 / 线速度
    //   lin_*  : 线速度 (body frame)；若不信任线速度可设极大方差
    //   ang_*  : 角速度 (body frame)；提供对 omega 状态的直接约束，消除旋转滞后
    node->declare_parameter("obs_cov.odom_twist_cov_lin_x",  1.0e9);   // 不信任
    node->declare_parameter("obs_cov.odom_twist_cov_lin_y",  1.0e9);   // 不信任
    node->declare_parameter("obs_cov.odom_twist_cov_lin_z",  1.0e9);   // 不信任
    node->declare_parameter("obs_cov.odom_twist_cov_ang_x",  0.01);    // 信任 roll rate
    node->declare_parameter("obs_cov.odom_twist_cov_ang_y",  0.01);    // 信任 pitch rate
    node->declare_parameter("obs_cov.odom_twist_cov_ang_z",  0.01);    // 信任 yaw rate

    // /localization_pose → PoseWithCovarianceStamped (global pose, no velocity)
    // 策略: 信任 x/y/z/yaw，不信任 roll/pitch（全局位姿无重力对齐参考）
    static std::atomic<bool> localizer_initialized{false};
    auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/localization_pose",
        10,
        [localizer, node](const geometry_msgs::msg::PoseStamped& msg) {

            // output delay millisecond
            auto delay = node->now() - rclcpp::Time(msg.header.stamp);
            // RCLCPP_INFO_STREAM(node->get_logger(), "Global pose delay: " << delay.seconds() * 1000.0 << " ms");

            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = msg.header;
            pose_msg.pose.pose = msg.pose;
            pose_msg.pose.covariance.fill(0.0);
            pose_msg.pose.covariance[0]  = node->get_parameter("obs_cov.global_pose_cov_x").as_double();
            pose_msg.pose.covariance[7]  = node->get_parameter("obs_cov.global_pose_cov_y").as_double();
            pose_msg.pose.covariance[14] = node->get_parameter("obs_cov.global_pose_cov_z").as_double();
            pose_msg.pose.covariance[21] = node->get_parameter("obs_cov.global_pose_cov_roll").as_double();
            pose_msg.pose.covariance[28] = node->get_parameter("obs_cov.global_pose_cov_pitch").as_double();
            pose_msg.pose.covariance[35] = node->get_parameter("obs_cov.global_pose_cov_yaw").as_double();

            if (!localizer_initialized.load()) {
                localizer->setInitialPose(pose_msg);
                localizer_initialized.store(true);
            } else {
                localizer->feedObservation(pose_msg);
            }
        }
    );

    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "Odometry",
        10,
        [localizer, node](const nav_msgs::msg::Odometry& msg) {
            if (!localizer_initialized.load()) return; // wait for initial pose

            const double delay_ms = (node->now() - rclcpp::Time(msg.header.stamp)).seconds() * 1000.0;
            auto& clock = *node->get_clock();
            RCLCPP_INFO_THROTTLE(node->get_logger(), clock, 2000, "[odom] receive delay: %.2f ms", delay_ms);

            // ── Absolute pose (used for roll/pitch correction only) ─────────────
            // 策略: 仅信任 roll/pitch（IMU重力对齐可靠），x/y/z/yaw 设极大协方差
            // geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            // pose_msg.header    = msg.header;
            // pose_msg.pose.pose = msg.pose.pose;

            // pose_msg.pose.covariance.fill(0.0);
            // pose_msg.pose.covariance[0]  = node->get_parameter("obs_cov.odom_pose_cov_x").as_double();
            // pose_msg.pose.covariance[7]  = node->get_parameter("obs_cov.odom_pose_cov_y").as_double();
            // pose_msg.pose.covariance[14] = node->get_parameter("obs_cov.odom_pose_cov_z").as_double();
            // pose_msg.pose.covariance[21] = node->get_parameter("obs_cov.odom_pose_cov_roll").as_double();
            // pose_msg.pose.covariance[28] = node->get_parameter("obs_cov.odom_pose_cov_pitch").as_double();
            // pose_msg.pose.covariance[35] = node->get_parameter("obs_cov.odom_pose_cov_yaw").as_double();

            // localizer->feedObservation(pose_msg);

            // ── Twist (velocity) observation — 直接注入角速度/线速度 ─────────────
            // 读取动态参数（支持运行时 ros2 param set 调整）
            const double tw_lin_x = node->get_parameter("obs_cov.odom_twist_cov_lin_x").as_double();
            const double tw_lin_y = node->get_parameter("obs_cov.odom_twist_cov_lin_y").as_double();
            const double tw_lin_z = node->get_parameter("obs_cov.odom_twist_cov_lin_z").as_double();
            const double tw_ang_x = node->get_parameter("obs_cov.odom_twist_cov_ang_x").as_double();
            const double tw_ang_y = node->get_parameter("obs_cov.odom_twist_cov_ang_y").as_double();
            const double tw_ang_z = node->get_parameter("obs_cov.odom_twist_cov_ang_z").as_double();

            // 构造 TwistWithCovarianceStamped，时间戳与 odom 原始帧一致，
            // 保留延迟补偿路径（delay_step 由 timerCallback 计算）。
            geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
            twist_msg.header = msg.header;
            twist_msg.twist.twist.linear.x  = msg.twist.twist.linear.x;
            twist_msg.twist.twist.linear.y  = msg.twist.twist.linear.y;
            twist_msg.twist.twist.linear.z  = msg.twist.twist.linear.z;
            twist_msg.twist.twist.angular.x = msg.twist.twist.angular.x;
            twist_msg.twist.twist.angular.y = msg.twist.twist.angular.y;
            twist_msg.twist.twist.angular.z = msg.twist.twist.angular.z;

            // 协方差矩阵布局: [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z]，仅对角项有效
            twist_msg.twist.covariance.fill(0.0);
            twist_msg.twist.covariance[0]  = tw_lin_x;
            twist_msg.twist.covariance[7]  = tw_lin_y;
            twist_msg.twist.covariance[14] = tw_lin_z;
            twist_msg.twist.covariance[21] = tw_ang_x;
            twist_msg.twist.covariance[28] = tw_ang_y;
            twist_msg.twist.covariance[35] = tw_ang_z;

            localizer->feedObservation(twist_msg);
        }
    );


    // --------------------------------- Launch Mapping --------------------------------------
    auto map_server = finenav_engine->createMapResource<GridMap<Voxel>>(
        "my_map_server", 10.0, true, 100.0);

    // A* 使用的二维 OccupancyGrid 地图资源（数据注入属于 Mapping）
    auto occ_grid_map_server = finenav_engine->createMapResource<finenav::OccGrid2DMap>(
        "occ_grid_2d_map_server", 5.0, false);





    // ============================================================
    // 可视化点云 Helpers（由 main 持有，生命周期覆盖所有使用方）
    // ============================================================

    // 1. /map_cloud : 地图中有效 Voxel 的高度点云（白色）
    auto map_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 10);
    auto map_cloud_helper = std::make_shared<finenav_utils::CloudPublishHelper>();
    map_cloud_helper->configure(map_cloud_pub, true, "map");

    // 2. passability_map : 地形可通行性（绿=可通行, 红=障碍）
    auto passability_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("passability_map", 10);
    auto passability_helper = std::make_shared<finenav_utils::CloudPublishHelper>();
    passability_helper->configure(passability_pub, true, "map");

    // 3. costmap_cloud : 膨胀后代价图（绿=低代价, 红=高代价）
    auto costmap_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("costmap_cloud", 10);
    auto costmap_helper = std::make_shared<finenav_utils::CloudPublishHelper>();
    costmap_helper->configure(costmap_pub, true, "map");

    // 4. local_map : SimpleGridMapView 局部地图（按代价着色）
    auto local_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 10);
    auto local_map_helper = std::make_shared<finenav_utils::CloudPublishHelper>();
    local_map_helper->configure(local_map_pub, false, "map");

    // 5. ground_cloud : 地面高度点云（青色）
    auto ground_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_cloud", 10);
    auto ground_helper = std::make_shared<finenav_utils::CloudPublishHelper>();
    ground_helper->configure(ground_pub, true, "map");

    // ============================================================

    auto terrain_analyzer = std::make_shared<TerrainAnalyzer>(
        map_server, node, passability_helper, costmap_helper, ground_helper);

    using Clock = std::chrono::steady_clock;
    using Ms    = std::chrono::duration<double, std::milli>;
    auto cycle_t0 = std::make_shared<Clock::time_point>();

    map_server->registerPreUpdateHook([stvl_manager, localizer, cycle_t0](GridMap<Voxel> & map) {
        *cycle_t0 = Clock::now();
        const auto & state = localizer->getState();
        Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
        T_map_body.linear() = state.R.matrix();
        T_map_body.translation() = state.p;
        stvl_manager->PruneExpiredCells(map, T_map_body);
    });

    map_server->registerPostUpdateHook([node, map_cloud_helper, terrain_analyzer, localizer, cycle_t0](GridMap<Voxel> & map) {

        terrain_analyzer->update(localizer->getState().p.z() - terrain_analyzer->getGroundZOffset(), map);

        // /map_cloud 可视化：所有有效 Voxel，高度映射到颜色（蓝低红高，简化为白色）
        if (map_cloud_helper->hasSubscribers()) {
            std::vector<Index> valid_indices;
            map.selectCellsByCondition(valid_indices, [](const Voxel & v) {
                return std::isfinite(v.z_height);
            });
            if (!valid_indices.empty()) {
                map_cloud_helper->reserve(valid_indices.size());
                for (const auto & idx : valid_indices) {
                    const auto pos = map.getPosition(idx);
                    const auto & cell = map.at(idx);
                    map_cloud_helper->addPoint(
                        static_cast<float>(pos.x()),
                        static_cast<float>(pos.y()),
                        cell.z_height,
                        {255, 255, 255}); // 白色
                }
                map_cloud_helper->publish(node->now());
            }
        }
    });

    MapServer<GridMap<Voxel>>::SourceBufferPolicy policy;
    policy.max_buffer_size = 50000;
    policy.observation_keep_time_sec = 2.0;

    auto laser_injector = map_server->addUpdateSource<pcl::PointCloud<pcl::PointXYZ>>(
        "front_laser",
        policy,
        [stvl_manager, localizer](
            const pcl::PointCloud<pcl::PointXYZ> & cloud,
            const rclcpp::Time & stamp,       // 点云的原始采集时间戳
            GridMap<Voxel> & map)
        {
            // 使用点云采集时刻的历史位姿，消除点云与位姿时间不同步导致的旋转漂移。
            // 若该时刻已超出历史窗口或 localizer 尚未初始化，则降级到最新位姿。
            const NavStateD state = [&]() -> NavStateD {
                if (auto s = localizer->getStateAt(stamp)) {
                    return *s;
                }
                return localizer->getState();
            }();

            Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
            T_map_body.linear()      = state.R.matrix();
            T_map_body.translation() = state.p;
            stvl_manager->InsertPointCloud(cloud, map, T_map_body);
        });

    auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body",
        10,
        [node, laser_injector](const sensor_msgs::msg::PointCloud2 & msg) {
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl::fromROSMsg(msg, pcl_cloud);
            laser_injector(std::move(pcl_cloud), rclcpp::Time(msg.header.stamp));
        });

    MapServer<finenav::OccGrid2DMap>::SourceBufferPolicy occ_policy;
    occ_policy.max_buffer_size = 1;
    occ_policy.observation_keep_time_sec = 0.0;

    auto occ_map_injector = occ_grid_map_server->addUpdateSource<nav_msgs::msg::OccupancyGrid>(
        "occ_grid_source",
        occ_policy,
        [](const nav_msgs::msg::OccupancyGrid& msg,
           const rclcpp::Time&,
           finenav::OccGrid2DMap& map) {
            map.update(msg);
        });

    auto occ_map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(1).transient_local().reliable(),
        [occ_map_injector](const nav_msgs::msg::OccupancyGrid& msg) {
            occ_map_injector(nav_msgs::msg::OccupancyGrid(msg), rclcpp::Time(msg.header.stamp));
        });


    // --------------------------------- Launch Planning --------------------------------------
    using AstarPlanners = PlannerSet<finenav::AstarPathSearch>;
    auto astar_layer = finenav_engine->createControlLayer<AstarPlanners>(
        "astar_layer", finenav::SingleShotPolicy{});

    auto astar_path_pub = node->create_publisher<nav_msgs::msg::Path>("/astar_path", 10);

    astar_layer->set_on_post_plan(
        [astar_path_pub](finenav::PostPlanContext& ctx) -> bool {
            if (ctx.is_success) {
                nav_msgs::msg::Path path_msg;
                path_msg.header = ctx.result_traj.header;
                if (path_msg.header.frame_id.empty()) {
                    path_msg.header.frame_id = "map";
                }
                path_msg.poses.reserve(ctx.result_traj.poses.size());
                for (const auto& pose : ctx.result_traj.poses) {
                    geometry_msgs::msg::PoseStamped ps;
                    ps.header = path_msg.header;
                    ps.pose = pose;
                    path_msg.poses.push_back(ps);
                }
                astar_path_pub->publish(path_msg);
            }
            return ctx.is_success;
        });

    node->declare_parameter("astar_map_view.lethal_radius",       0.20);
    node->declare_parameter("astar_map_view.inflation_radius",    0.50);
    node->declare_parameter("astar_map_view.cost_scaling_factor", 10.0);

    astar_layer->bind_map_view(
        occ_grid_map_server,
        [node](const finenav::OccGrid2DMap& map) {
            double lethal_radius       = node->get_parameter("astar_map_view.lethal_radius").as_double();
            double inflation_radius    = node->get_parameter("astar_map_view.inflation_radius").as_double();
            double cost_scaling_factor = node->get_parameter("astar_map_view.cost_scaling_factor").as_double();
            return std::make_shared<astar::OccGridMapView>(map, 50, lethal_radius, inflation_radius, cost_scaling_factor);
        });

    using MppiPlanners = PlannerSet<nav2_mppi_controller::MPPIController>;
    auto my_control_layer = finenav_engine->createControlLayer<MppiPlanners>(
        "mppi_layer", finenav::TrackingPolicy{10.0});

    // /cmd_vel publisher — 由 post_plan 回调驱动，在每次规划成功后取第一步速度指令
    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // [DEBUG] 发布裁减后的参考轨迹，供 RViz 可视化验证
    auto trimmed_path_pub = node->create_publisher<nav_msgs::msg::Path>("/trimmed_ref_path", 10);

    // 发布 SmoothPath 平滑后（TrimByDistance 前）的完整参考路径
    auto smooth_path_pub = node->create_publisher<nav_msgs::msg::Path>("/smooth_path", 10);

    Region3D map_bounds;
    {
        map_bounds = map_server->getLockedReadView()->getWindowBounds();
    }

    my_control_layer->set_on_pre_plan(
    [map_bounds, map_server, localizer, node, trimmed_path_pub, smooth_path_pub](finenav::PrePlanContext& ctx) -> bool {
        // 1. Advance trajectory start to the waypoint nearest to the robot
        const auto& rp = ctx.robot_state.pose.position;
        ctx.ref_traj | finenav::SeekToNearestPoint({rp.x, rp.y, rp.z});

        // 2. Trim trailing waypoints beyond the maximum tracking distance
        //    (use the shorter dimension of the map as a conservative bound)
        // const auto bounds_size = map_bounds.max() - map_bounds.min();
        // const double max_dist = std::min(bounds_size.x(), bounds_size.y());
        // ctx.ref_traj | finenav::TrimByDistance(max_dist);

        // 3. 平滑参考轨迹（拓扑规划输出的折线做拉普拉斯平滑）
        ctx.ref_traj | finenav::SmoothPath();

        // 发布平滑后、裁剪前的完整路径
        {
            nav_msgs::msg::Path smooth_msg;
            smooth_msg.header.stamp = node->now();
            smooth_msg.header.frame_id = "map";
            for (const auto& pose : ctx.ref_traj.poses) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = smooth_msg.header;
                ps.pose = pose;
                smooth_msg.poses.push_back(ps);
            }
            smooth_path_pub->publish(smooth_msg);
        }

        // 4. 只保留前方 3m 的轨迹点，MPPI 窗口够用即可
        ctx.ref_traj | finenav::TrimByDistance(3.0);

        // [DEBUG] 发布裁减后的参考轨迹供 RViz 可视化验证
        {
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = node->now();
            path_msg.header.frame_id = "map";
            for (const auto& pose : ctx.ref_traj.poses) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path_msg.header;
                ps.pose = pose;
                path_msg.poses.push_back(ps);
            }
            trimmed_path_pub->publish(path_msg);
        }

        return true;
    });

    my_control_layer->set_on_post_plan(
        [cmd_vel_pub](finenav::PostPlanContext& ctx) -> bool {
            if (ctx.is_success &&
                !ctx.result_traj.twists.empty())
            {
                cmd_vel_pub->publish(ctx.result_traj.twists[0]);
            }
            return ctx.is_success;
        });

    // local_map_helper 捕获到 builder lambda，每次构造 SimpleGridMapView 时传入
    // robot_state_provider 和 should_stop_predicate 已由 FineNavEngine::createControlLayer() 自动注入。
    my_control_layer->bind_map_view(
        map_server,
        [node, terrain_analyzer, local_map_helper](const GridMap<Voxel>& map) {
            return std::make_shared<SimpleGridMapView>(map, node, terrain_analyzer, local_map_helper);
        });


    finenav_engine->registerPipeline(
        FineNavEngine::PipelineIO{
            .input_topic = "",
            .output_topic = "astar_layer_plan",
        },
        astar_layer);

    finenav_engine->registerPipeline(
        FineNavEngine::PipelineIO{
            .input_topic = "astar_layer_plan",
            .output_topic = "mppi_layer_plan",
        },
        my_control_layer);


    // --------------------------------- Main Loop --------------------------------------------

    // 用独立 OS 线程驱动 node 的 spin，彻底解耦回调执行与主循环调度。
    // odom 回调由该线程立即分发，不受主线程 CPU 压力或调度窗口影响。
    auto node_thread = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    node_thread->add_node(node);
    std::thread spin_thread([&node_thread]() { node_thread->spin(); });

    rclcpp::Rate loop_rate(200); // 200Hz
    auto nav_state_pub = node->create_publisher<nav_msgs::msg::Odometry>("nav_state", 10);
    auto nav_state_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("nav_state_pose", 10);
    auto nav_state_twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("nav_state_twist", 10);

    while (rclcpp::ok()) {
        auto robot_state = localizer->getState();

        // Initialize with IIFE
        const auto nav_state_msg = [&node, &robot_state] {
            nav_msgs::msg::Odometry nav_state_msg;
            nav_state_msg.header.stamp = node->now();
            nav_state_msg.header.frame_id = "map";
            nav_state_msg.child_frame_id = "base_link";

            nav_state_msg.pose.pose.position.x = robot_state.p.x();
            nav_state_msg.pose.pose.position.y = robot_state.p.y();
            nav_state_msg.pose.pose.position.z = robot_state.p.z();

            auto quat = robot_state.R.unit_quaternion();
            nav_state_msg.pose.pose.orientation.x = quat.x();
            nav_state_msg.pose.pose.orientation.y = quat.y();
            nav_state_msg.pose.pose.orientation.z = quat.z();
            nav_state_msg.pose.pose.orientation.w = quat.w();

            nav_state_msg.twist.twist.linear.x = robot_state.v.x();
            nav_state_msg.twist.twist.linear.y = robot_state.v.y();
            nav_state_msg.twist.twist.linear.z = robot_state.v.z();
            nav_state_msg.twist.twist.angular.x = robot_state.omega.x();
            nav_state_msg.twist.twist.angular.y = robot_state.omega.y();
            nav_state_msg.twist.twist.angular.z = robot_state.omega.z();

            return nav_state_msg;
        }();
        nav_state_pub->publish(nav_state_msg);

        // nav_state_pose
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = nav_state_msg.header;
        pose_msg.pose   = nav_state_msg.pose.pose;
        nav_state_pose_pub->publish(pose_msg);

        // nav_state_twist
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header = nav_state_msg.header;
        twist_msg.twist  = nav_state_msg.twist.twist;
        nav_state_twist_pub->publish(twist_msg);


  		// spin_some 已移至独立线程，主循环只做 nav_state 发布与限速
        loop_rate.sleep();
    }

    node_thread->cancel();
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}
