// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_engine/finenav_engine.hpp"
#include "finenav_core/map/map_server.hpp"
#include "simpleTestMap.hpp"
#include "simpleTestPlanner.hpp"

using namespace finenav;

struct DummyLaserScan {
    int seq;
    float range;
};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("finenav_demo");

using GlobalPlanners = PlannerSet<SimpleTestPlanner>;
using LocalLayers = PlannerSet<SimpleTestPlanner>;
using ReactivePlanners = PlannerSet<SimpleTestPlanner>;

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("finenav_demo");

    auto finenav_ptr = std::make_shared<FineNavEngine>(node);

    auto global_layer = finenav_ptr->createControlLayer<GlobalPlanners>("global_layer");
    auto local_layer = finenav_ptr->createControlLayer<LocalLayers>("local_layer");
    auto reactive_layer = finenav_ptr->createControlLayer<ReactivePlanners>("reactive_layer");

    finenav_ptr->registerPipeline(FineNavEngine::PipelineIO{}, global_layer, local_layer, reactive_layer);

    // ==============================================================================
    // 测试 FineNavLocalizer
    // ==============================================================================
    finenav_ptr->getLocalizer(); // 【注意：目前存在小bug，必须将这个api在createMapResource前调用一次】


    // ==============================================================================
    // 测试 MapServer 的完整功能 (Hook机制 + 数据源Push模式)
    // ==============================================================================

    // 1. 初始化一个地图服务器，指定更新频率 1.0Hz
    auto map_server_ptr = finenav_ptr->createMapResource<SimpleTestMap>(
        "test_map_server",
        1.0,   // update_rate_hz
        true,  // enable_shift_window
        50.0   // shift_rate_hz
    );

    // 2. 注册预处理 Hook (在应用融合数据前执行)
    map_server_ptr->registerPreUpdateHook([](SimpleTestMap& map) {
        RCLCPP_INFO(LOGGER, ">> [Hook] PreUpdateHook triggered. Preparing map...");
    });

    // 3. 注册后处理 Hook (在应用融合数据后执行)
    map_server_ptr->registerPostUpdateHook([](const SimpleTestMap& map) {
        RCLCPP_INFO(LOGGER, "<< [Hook] PostUpdateHook triggered. Map updating complete.");
    });

    // 4. 初始化策略并注册数据注射器
    MapServer<SimpleTestMap>::SourceBufferPolicy policy;
    policy.max_buffer_size = 10;
    policy.observation_keep_time_sec = 2.0;

    auto laser_injector = map_server_ptr->addUpdateSource<DummyLaserScan>(
        "front_laser",
        policy,
        [](const DummyLaserScan& msg, const rclcpp::Time& /*stamp*/, SimpleTestMap& map) {
            RCLCPP_INFO(LOGGER, "   [Fuser] Fusing DummyLaserScan data: Seq = %d, Range = %.2f", msg.seq, msg.range);
            (void)map; // 防止 unused parameter 警告
        }
    );

    // 5. 模拟一个高频的传感器硬件或话题回调，产生数据并持续推入注射器
    std::shared_ptr<int> mock_seq_count = std::make_shared<int>(0);
    auto mock_sensor_timer = node->create_wall_timer(std::chrono::milliseconds(200), [node, laser_injector, mock_seq_count]() {
        DummyLaserScan mock_scan_data{};
        mock_scan_data.seq = (*mock_seq_count)++;
        mock_scan_data.range = 5.0f;

        // 我们只需把传感器数据无脑塞进去，MapServer内部会自动保护并等待周期来临
        // 强制采用 std::move 提供右值，明确所有权移交，注入后数据作废
        laser_injector(std::move(mock_scan_data), node->now());
    });

    // 6. 添加一个定时器，模拟用户主动去访问 Map
    auto mock_user_timer = node->create_wall_timer(std::chrono::milliseconds(500), [&map_server_ptr]() {
        // 用户获取带有读锁的视图，极度丝滑
        auto map_view = map_server_ptr->getLockedReadView();
        // 因为操作符重载，等价于操作 const SimpleTestMap 对象本身的方法，自动线程安全
        RCLCPP_INFO(LOGGER, "   [User] Map accessed safely. LockedReadView protects the thread.");
    });

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
