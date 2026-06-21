// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/behavior_tree_engine.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace finenav {

BehaviorTreeEngine::BehaviorTreeEngine(rclcpp::Node::SharedPtr node)
  : logger_(node->get_logger())
{
    // 初始化 RosNodeParams，这是 BT.ROS2 依赖注入的核心
    ros_params_.nh = node;
    ros_params_.default_port_value = ""; // 默认不指定 topic，强制 XML 指定
    ros_params_.server_timeout = std::chrono::milliseconds(1000);
    ros_params_.wait_for_server_timeout = std::chrono::milliseconds(500);
}

void BehaviorTreeEngine::registerPlugins(const std::vector<std::string>& plugins)
{

    RCLCPP_INFO(logger_, "FineNav Engine: Loading plugins from %zu sources...", plugins.size());

    for (const auto& plugin_param : plugins) {
        const auto [pkg_name, sub_folder] = parseParamPath(plugin_param);
        if (pkg_name.empty()) { continue; } // skip invalid plugins directories

        std::filesystem::path plugin_directory;
        try {
            // Plugins are usually installed in "[ws]/install/[pkg_name]/lib/"
            const auto prefix = ament_index_cpp::get_package_prefix(pkg_name);
            plugin_directory = std::filesystem::path(prefix) / sub_folder;
        }
        catch (const ament_index_cpp::PackageNotFoundError& e) {
            RCLCPP_ERROR(logger_, "Plugin loading failed: Package '%s' not found.", pkg_name.c_str());
            continue;
        }
        if (!std::filesystem::exists(plugin_directory)) {
            RCLCPP_WARN(logger_, "Plugin directory not found: %s", plugin_directory.c_str());
            continue;
        }

        // Discovery and load .so
        using std::filesystem::recursive_directory_iterator;
        int loaded_count = 0;
        for(const auto& entry : recursive_directory_iterator(plugin_directory))
        {
            if(entry.path().extension() == ".so")
            {
                if (loadPlugin(factory_, entry.path(), ros_params_)) {
                    loaded_count++;
                }
            }
        }
        RCLCPP_INFO(logger_, "  -> Scanned '%s': Loaded %d plugins.", plugin_directory.c_str(), loaded_count);
    }
}

void BehaviorTreeEngine::registerBehaviorTrees(const std::vector<std::string>& tree_dirs)
{
    for (const auto& tree_dir_param : tree_dirs) {
        const auto [pkg_name, sub_folder] = parseParamPath(tree_dir_param);
        if (pkg_name.empty()) { continue; } // skip invalid plugins directories

        std::filesystem::path tree_directory;
        try {
            // BehaviorTress are usually installed in "[ws]/install/[pkg_name]/share/[pkg_name]/"
            const auto share_dir = ament_index_cpp::get_package_share_directory(pkg_name);
            tree_directory = std::filesystem::path(share_dir) / sub_folder;
        }
        catch (const ament_index_cpp::PackageNotFoundError& e) {
            RCLCPP_ERROR(logger_, "BT loading failed: Package '%s' not found.", pkg_name.c_str());
            continue;
        }
        if (!std::filesystem::exists(tree_directory)) {
            RCLCPP_WARN(logger_, "Plugin directory not found: %s", tree_directory.c_str());
            continue;
        }

        // Discovery and load .xml
        using std::filesystem::recursive_directory_iterator;
        int loaded_count = 0;
        for(const auto& entry : recursive_directory_iterator(tree_directory))
        {
            if (loadBehaviorTree(factory_, entry.path())) {
                loaded_count++;
            }
        }
        RCLCPP_INFO(logger_, "  -> Scanned '%s': Loaded %d trees.", tree_directory.c_str(), loaded_count);
    }
}

bool BehaviorTreeEngine::loadPlugin(
    BT::BehaviorTreeFactory& factory,
    const std::filesystem::path& file_path,
    BT::RosNodeParams params) {

    const auto filename = file_path.filename();
    try
    {
        BT::SharedLibrary loader(file_path.string());
        if(loader.hasSymbol(BT::PLUGIN_SYMBOL))
        {
            typedef void (*Func)(BT::BehaviorTreeFactory&);
            auto func = (Func)loader.getSymbol(BT::PLUGIN_SYMBOL);
            func(factory);
        }
        else if(loader.hasSymbol(BT::ROS_PLUGIN_SYMBOL))
        {
            typedef void (*Func)(BT::BehaviorTreeFactory&, const BT::RosNodeParams&);
            auto func = (Func)loader.getSymbol(BT::ROS_PLUGIN_SYMBOL);
            func(factory, params);
        }
        else
        {
            RCLCPP_ERROR(logger_, "Failed to load Plugin from file: %s.", filename.c_str());
            return false;
        }
        RCLCPP_INFO(logger_, "Loaded ROS Plugin: %s", filename.c_str());
        return true;
    }
    catch(const std::exception& ex)
    {
        RCLCPP_ERROR(logger_, "Failed to load ROS Plugin: %s \n %s", filename.c_str(),
                     ex.what());
        return false;
    }
}

bool BehaviorTreeEngine::loadBehaviorTree(
    BT::BehaviorTreeFactory& factory,
    const std::filesystem::path& file_path) {

    try
    {
        factory.registerBehaviorTreeFromFile(file_path.string());
        RCLCPP_INFO(logger_, "Loaded BehaviorTree: %s", file_path.filename().c_str());
        return true;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "Failed to load BehaviorTree: %s \n %s",
                     file_path.filename().c_str(), e.what());
        return false;
    }
}

BT::BehaviorTreeFactory& BehaviorTreeEngine::factory() {
    return factory_;
}

std::pair<std::string, std::string> BehaviorTreeEngine::parseParamPath(const std::string& input) {
    auto pos = input.find('/');
    if (pos == std::string::npos) {
        return {input, ""}; // only package name
    }
    return {input.substr(0, pos), input.substr(pos + 1)};
}

}
