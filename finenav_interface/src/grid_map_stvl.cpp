// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-4-2.
//
#include "grid_map.hpp"
#include "simpleTestPlanner.hpp"
#include "finenav_engine/finenav_engine.hpp"
#include "finenav_core/map/map_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "frustum3d.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

using namespace finenav;

struct Pointcloud {
    float x;
    float y;
    float z;
};

// 体素中存储：z 轴高度、是否静态、最近一次更新时间戳（ROS2 时间）
struct Voxel {
    float        z_height  = NAN;
    bool         is_static = false;
    rclcpp::Time timestamp{};   // 使用 node->get_clock()->now() 赋值 //TODO:改成位数更小的数据结构
};
// ============================ 动态栅格管理相关 ============================

// 使用 Position 作为 key 的哈希器和相等判断，只在本 demo 中使用
struct PositionHash
{
    std::size_t operator()(const Position & p) const noexcept
    {
        // Position 是 Eigen::Vector3d，直接对三维坐标做 hash
        std::size_t hx = std::hash<double>{}(p.x());
        std::size_t hy = std::hash<double>{}(p.y());
        std::size_t hz = std::hash<double>{}(p.z());

        auto hash_combine = [](std::size_t seed, std::size_t v) {
            seed ^= v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            return seed;
        };

        std::size_t seed = 0;
        seed = hash_combine(seed, hx);
        seed = hash_combine(seed, hy);
        seed = hash_combine(seed, hz);
        return seed;
    }
};

struct PositionEqual
{
    bool operator()(const Position & a, const Position & b) const noexcept
    {
        // 由于 Position 将通过 GridMap::getPosition(Index) 得到，理论上是栅格中心的精确坐标，
        // 这里直接用完全相等判断。如果将来引入插值，可在此处添加容差。
        return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

using DynamicPosSet = std::unordered_set<Position, PositionHash, PositionEqual>;

// 全局（本文件作用域）动态栅格集合与超时时间，仅用于 demo
static DynamicPosSet g_dynamic_cells;          // 存储动态栅格的中心 Position
// 基础超时阈值（视野外或普通情况下）：单位：秒
static double        g_decay_time_sec = 10.0;
static double        g_decay_time_fov_sec = 1.0;           //TODO：：只是简单的调整生命时间，是否有需要是加速衰减速度

static const rclcpp::Logger LOGGER = rclcpp::get_logger("finenav_demo");

using GlobalPlanners = PlannerSet<SimpleTestPlanner>;
using LocalLayers = PlannerSet<SimpleTestPlanner>;
using ReactivePlanners = PlannerSet<SimpleTestPlanner>;
using std::chrono_literals::operator"" ms;

bool numisfinite(float x) {
    if(x==std::numeric_limits<float>::infinity() || x==-std::numeric_limits<float>::infinity() || std::isnan(x)) {
        return false;
    }
    return true;
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("finenav_demo");

    auto finenav_ptr = std::make_shared<FineNavEngine>(node);

    auto global_layer = finenav_ptr->createControlLayer<GlobalPlanners>("global_layer");
    auto local_layer = finenav_ptr->createControlLayer<LocalLayers>("local_layer");
    auto reactive_layer = finenav_ptr->createControlLayer<ReactivePlanners>("reactive_layer");

    finenav_ptr->registerPipeline(FineNavEngine::PipelineIO{}, global_layer, local_layer, reactive_layer);
    auto map_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 10);
    auto frustum_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/frustum_voxels", 10);
    // ==============================================================================
    // 测试 MapServer 的完整功能 (Hook机制 + 数据源Push模式)
    // ==============================================================================

    // 1. 初始化一个地图服务器，指定更新频率
    MapServer<GridMap<Voxel>> map_server(node, "test_map_server", 10.0);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_buffer->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(node->get_node_base_interface(),
                                                  node->get_node_timers_interface()));

    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // 2. 注册预处理 Hook (在应用融合数据前执行)
    map_server.registerPreUpdateHook([node, tf_buffer, frustum_cloud_pub](GridMap<Voxel>& map) {
        using finenav::geometry::Frustum3D;
        using finenav::geometry::FrustumParams3D;
        using finenav::geometry::FrustumSensorPose;
        using finenav::geometry::FrustumRange;
        using finenav::geometry::FrustumAngles;

        rclcpp::Time now = node->get_clock()->now();

        // 1. 获取 base_lidar 在 map 下的变换
        Eigen::Matrix4d T_ws = Eigen::Matrix4d::Identity();
        try {
            geometry_msgs::msg::TransformStamped tf_stamped =
                tf_buffer->lookupTransform("map", "base_lidar", tf2::TimePointZero);
            Eigen::Isometry3d iso = tf2::transformToEigen(tf_stamped.transform);
            T_ws = iso.matrix();
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
            // 保持 T_ws = Identity
        }

        FrustumSensorPose sensor_pose;
        sensor_pose.T_ws = T_ws;

        // 视锥参数
        FrustumRange  range;
        range.min_range = 0.0;
        range.max_range = 50.0;

        FrustumAngles fov;
        fov.min_angle   = 0.0;
        fov.max_angle = M_PI / 3.0;

        FrustumParams3D params{sensor_pose, range, fov};
        Frustum3D frustum(params);

        // 遍历所有当前记录为“动态”的栅格中心 Position，根据时间戳 + 是否在视锥内判断是否需要清除
        for (auto it = g_dynamic_cells.begin(); it != g_dynamic_cells.end(); ) {
            const Position & pos = *it;

            // 如果位置已不在地图有效范围内，直接移除
            if (!map.isInside(pos)) {
                it = g_dynamic_cells.erase(it);
                continue;
            }

            Voxel & cell = map.atPosition(pos);

            // 使用 ROS2 时间计算持续时间
            rclcpp::Duration age = now - cell.timestamp;
            const double age_sec = age.seconds();

            // 首先：若年龄已经超过基础阈值，直接删除，无需再做视锥判断
            if (age_sec > g_decay_time_sec) {
                cell = Voxel{NAN, false, rclcpp::Time{}};
                it   = g_dynamic_cells.erase(it);
                continue;
            }

            // 基于视锥判断，若在视锥内，则使用更短的寿命（加速清除）
            bool in_fov = frustum.contains(pos.x(), pos.y(), pos.z());
            double active_decay = in_fov ? g_decay_time_fov_sec : g_decay_time_sec;

            if (age_sec > active_decay) {
                cell = Voxel{NAN, false, rclcpp::Time{}};
                it   = g_dynamic_cells.erase(it);
            } else {
                ++it;
            }
        }

    });

    // 3. 注册后处理 Hook (在应用融合数据后执行)
    map_server.registerPostUpdateHook(
        [node, map_cloud_pub](const GridMap<Voxel>& map)
        {
            auto now = node->now();
            std::vector<Index> valid_indices;
            map.selectCellsByCondition(valid_indices, [](const Voxel& v) {
                return std::isfinite(v.z_height);
            });
            // std::cout<<"valid_indices size: "<<valid_indices.size()<<std::endl;
            if (valid_indices.empty()) return;

            sensor_msgs::msg::PointCloud2 cloud;
            cloud.header.stamp = now;
            cloud.header.frame_id = "map";
            cloud.height = 1;
            cloud.width = valid_indices.size();

            sensor_msgs::PointCloud2Modifier modifier(cloud);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(valid_indices.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

            for (const auto& idx : valid_indices) {
                const auto pos = map.getPosition(idx);
                const auto& cell = map.at(idx);

                *iter_x = pos.x();
                *iter_y = pos.y();
                *iter_z = cell.z_height;

                ++iter_x;
                ++iter_y;
                ++iter_z;
            }

            map_cloud_pub->publish(cloud);
            RCLCPP_INFO(LOGGER, "<< [Hook] PostUpdateHook triggered.");

        }
    );

    // 4. 初始化策略并注册数据注射器
    MapServer<GridMap<Voxel>>::SourceBufferPolicy policy;
    policy.max_buffer_size = 50000;
    policy.observation_keep_time_sec = 2.0;
    auto laser_injector = map_server.addUpdateSource<Pointcloud>(
        "front_laser",
        policy,
        [node](const Pointcloud& msg, const rclcpp::Time& stamp, GridMap<Voxel>& map) {
            rclcpp::Time now = node->get_clock()->now();
            // 演示：假设激光击中地图中心 (0,0) 处的某个体素，只更新一次
            // 实际工程中应该根据传感器模型计算出命中位置
            Position hit_pos = Position(msg.x , msg.y, msg.z); // TODO: 根据 msg 计算实际命中位置

            if (!map.isInside(hit_pos)) {
                return;
            }

            Index  idx  = map.getIndex(hit_pos);
            Voxel & cell = map.at(idx);

            if (!cell.is_static) {
                cell.z_height  = msg.z;   // 这里用 range 代替 z，仅作示意
                cell.timestamp = now;         // 记录 ROS2 时间
                // 使用 Position 作为 key 记录为动态栅格（哈希表自动去重）
                Position center_pos = map.getPosition(idx);
                g_dynamic_cells.insert(center_pos);
            }
        }
    );

    // 新增：地图点云可视化发布器（手动触发发布）


    // 订阅点云并用 TF 过滤，保证 TF 可用时才处理点云
    auto cloud_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(node, "/cloud_for_lio");
    auto tf_filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *cloud_sub, *tf_buffer, "map", 10,
        node->get_node_logging_interface(), node->get_node_clock_interface(), 10ms);

    tf_filter->registerCallback(
        [node, laser_injector, tf_buffer](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
            geometry_msgs::msg::TransformStamped tf_stamped;
            try {
                tf_stamped = tf_buffer->lookupTransform("map", msg->header.frame_id, msg->header.stamp);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
                return;
            }
            Eigen::Isometry3d T_map_lidar = tf2::transformToEigen(tf_stamped.transform);

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
            rclcpp::Time now = node->now();

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                if(numisfinite(*iter_x) && numisfinite(*iter_y) && numisfinite(*iter_z)) {
                    Eigen::Vector3d p_lidar(*iter_x, *iter_y, *iter_z);
                    Eigen::Vector3d p_map = T_map_lidar * p_lidar;
                    Pointcloud p;
                    p.x = static_cast<float>(p_map.x());
                    p.y = static_cast<float>(p_map.y());
                    p.z = static_cast<float>(p_map.z());
                    laser_injector(std::move(p), now);
                }
            }
        }
    );

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

