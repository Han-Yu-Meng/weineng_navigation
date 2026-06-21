// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string_view>

#include <rclcpp/rclcpp.hpp>

// TODO: 改名 xxxx_marcos
namespace finenav {
// 不需要配置的默认模版
template <typename AlgoT>
struct AlgoConfigurator {
    static constexpr std::string_view name = "unknown_algo";

    // 返回一个智能指针，将生命周期转移给调用者
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr, AlgoT&) {
        // TODO: 这种情况应该警告，表示用户没有注册算法
        return nullptr; // 返回空指针，keep_alive 会自动忽略
    }
};
}

// TODO： 考虑 PARAM_NS 是否改名为 algo_name 或者 algo_id 之类的东西，只要能够对用户屏蔽参数概念
// TODO: 后续还需要考虑动态参数问题
// 另外，可能会有重名问题？
#define FINENAV_REGISTER_ALGO_CONFIG(ALGO_TYPE, PARAM_NS) \
namespace finenav { \
template <> \
struct AlgoConfigurator<ALGO_TYPE> { \
    static constexpr std::string_view name = #PARAM_NS; \
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr node, ALGO_TYPE& algo) { \
        /* 1. 创建监听器 */ \
        auto listener = std::make_shared<PARAM_NS::ParamListener>(node, #PARAM_NS); \
        /* 2. 初始配置 (立即生效) */ \
        algo.configure(listener->get_params()); \
        \
        /* 3. 注册参数更新回调 */ \
        listener->setUserCallback([&algo](const auto& params) { \
            algo.configure(params); \
        }); \
        \
        /* 4. 返回指针以维持生命周期 */ \
        return listener; \
    } \
}; \
}\

#define FINENAV_REGISTER_TEMPLATE_ALGO_CONFIG(TEMPLATE_ALGO, PARAM_NS) \
namespace finenav { \
template <typename... Args> \
struct AlgoConfigurator<TEMPLATE_ALGO<Args...>> { \
    static constexpr std::string_view name = #PARAM_NS; \
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr node, TEMPLATE_ALGO<Args...>& algo) { \
        /* 1. 创建监听器 */ \
        auto listener = std::make_shared<PARAM_NS::ParamListener>(node, #PARAM_NS); \
        /* 2. 初始配置 (立即生效) */ \
        algo.configure(listener->get_params()); \
        \
        /* 3. 注册参数更新回调 */ \
        listener->setUserCallback([&algo](const auto& params) { \
        algo.configure(params); \
        }); \
        \
        /* 4. 返回指针以维持生命周期 */ \
        return listener; \
    } \
};\
}\

