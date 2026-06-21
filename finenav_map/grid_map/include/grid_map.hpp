// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>

#include "finenav_map_grid_map/grid_map_params.hpp"
#include "finenav_util/algo_configurator.hpp"

#include "grid_map_math.hpp"
#include "grid_map_iterator.hpp"

namespace finenav { // TODO: 架构大改造，嵌套命名空间

/**
 * @brief 支持滚动更新的三维栅格地图类
 * @details 该地图维护两种索引方式，(1) Index 相对于地图原点的索引; (2) Position 相对于世界坐标系的坐标
 * @details 地图内部通过维护地图原点在世界坐标系中的位置，实现地图的滚动更新
 * @note 该栅格地图的原点总是在地图中心
 * @note 不支持地图原点的姿态变化
 */
template <typename T>
class GridMap {
public:
	using DataType = T;
    using ConfigType = grid_map::Params;

    // 1. 延迟初始化问题：构造之后不着急初始化，要处理 ROS 参数读取，再进行初始化配置
    // 2. 怎么读 ROS 参数，这是个问题: 弄个 ROSWrapper, 需要传递 node 进去，而后在合适时候调用 configure(node)，通过 generate_parameter_library
    // 读取预定义在 .yaml 中的参数，然后 GridMap 在内部处理这些 param 进行配置

	GridMap();

    /**
     * @brief 构造函数
     * @param length 栅格地图的尺寸，单位为米
     * @param resolution 栅格地图的分辨率，单位为米
     * @param origin 栅格地图的原点坐标
     * @note 所有地图数据均为零初始化
     */
    GridMap(const Length& length, const double& resolution, const Position& origin = Position::Zero());


    void configure(const ConfigType& params) {
        // TODO: 之后整，主要把 GridMap( Length, double, Poosition ) 的构造函数搬过来
        std::cout << "[GridMap] configure with params" << std::endl;

    // ===== 固定测试参数 =====
    resolution_ = 0.05;                     // 5cm
    length_ = Length(6.0, 6.0, 6.0);     // 10m x 10m x 3m
    origin_ = Position(0.0, 0.0, 0.0);

    // ===== 派生量 =====
    inv_resolution_ = 1.0 / resolution_;

    size_ = (length_.array() * inv_resolution_).ceil().cast<int>();

    // 保证是奇数（关键）
    size_ = size_.unaryExpr([](int v) {
        return (v % 2 == 0) ? v + 1 : v;
    });

    half_size_ = size_ / 2;

    // ===== 分配内存（核心）=====
    const size_t total_size =
        static_cast<size_t>(size_.x()) *
        static_cast<size_t>(size_.y()) *
        static_cast<size_t>(size_.z());

    data_.clear();
    data_.resize(total_size);

    // 初始化每个 voxel
    for (auto& cell : data_) {
        cell = T{};   // 你的 Voxel 默认是 NAN / false / 0
    }

    start_index_ = Index::Zero();

    // ===== debug 输出 =====
    std::cout << "[GridMap] resolution: " << resolution_ << std::endl;
    std::cout << "[GridMap] size: "
              << size_.x() << " x "
              << size_.y() << " x "
              << size_.z() << std::endl;
        std::cout << "[GridMap] total cells: " << total_size << std::endl;
    }

    /**
     * @brief 默认的构造函数
     */
    GridMap(const GridMap&) = default;
    GridMap& operator=(const GridMap&) = default;
    GridMap(GridMap&&) = default;
    GridMap& operator=(GridMap&&) = default;

    /**
     * @brief 析构函数
     */
    virtual ~GridMap() = default;

    /**
     * @brief 访问某个位置的栅格数据
     * @param position 世界坐标系下的位置
     * @return 对应位置的栅格数据
     * @throw
     */
    T& atPosition(const Position& position);

    /**
     * @brief 访问某个坐标点的栅格数据（常量版本）
     * @param position 世界坐标系下的位置
     * @return 对应位置的栅格数据
     */
    const T& atPosition(const Position& position) const;

    // TMP
    const DataType& access(const Position& position) const {
        return atPosition(position);
    }

    bool integrate(const Position& position, const DataType& data) {
        T& cell = atPosition(position);
        cell = data; // 简单覆盖更新 | 实际上，这里应该对应具体的更新逻辑，比如 raycast
        return true;
    }

    Region3D getWindowBounds() const {
        Position min_pos = getPosition(getMinIndex());
        Position max_pos = getPosition(-getMinIndex());
        return Region3D{min_pos, max_pos};
    }

    Position getWindowCenter() const {
        return origin_;
    }

    /**
     * @brief 访问某个index的栅格数据
     * @param index 栅格地图的索引
     * @return 对应的栅格数据
     */
    T& at(const Index& index);

    /**
     * @brief 访问某个index的栅格数据（常量版本）
     * @param index 栅格地图的索引
     * @return 对应的栅格数据
     */
    const T& at(const Index& index) const; // TODO：返回值类型改为 const T&


    /**
     * @brief 访问某个(x,y)索引上的一串栅格数据
     * @param x x索引
     * @param y y索引
     * @return 深拷贝的栅格数据向量，正确处理滚动缓冲区的 z 轴环绕
     */
    std::vector<T> getVoxelsAlongZ(const int& x, const int& y);


    /**
     * @brief 访问某个(x,y)索引上的一串栅格数据（常量版本）
     * @param x x索引
     * @param y y索引
     * @return 深拷贝的栅格数据向量，正确处理滚动缓冲区的 z 轴环绕
     */
    std::vector<T> getVoxelsAlongZ(const int& x, const int& y) const;

    /**
     * @brief 访问某个(x,y)索引上的一串栅格数据（低开销引用重载版本）
     * @param x x索引
     * @param y y索引
     * @param result 用于存放结果的外部预分配容器
     */
    void getVoxelsAlongZ(const int& x, const int& y, std::vector<T>& result) const;

    /**
     * @brief 清空地图数据
     */
    void clear();

    /**
     * @brief 移动地图，这会造成栅格数据的更新
     * @param position  地图原点移动到的新的位置
     * @return 如果地图实际发生移动返回true，否则返回false
     * @note 删除的栅格数据会被清除
     */
    bool moveTo(const Position& position);

    /**
     * @brief 移动地图，这会造成栅格数据的更新
     * @param[in] position 地图原点移动到的新的位置
     * @param[out] removed_region 移动过程中被删除的栅格数据及其对应的位置，仅当keep_removed为true时有效
     * @return 如果地图实际发生移动返回true，否则返回false
     */
    bool moveTo(const Position& position, std::vector<std::pair<Position, T>>& removed_region);

    /**
     * @brief 移动地图，这会造成栅格数据的更新
     * @param[in] position 地图原点移动到的新的位置
     * @param[in] keep_removed 是否保留被删除的栅格数据
     * @param[out] removed_region 移动过程中被删除的栅格数据及其对应的位置，仅当keep_removed为true时有效
     * @return 如果地图实际发生移动返回true，否则返回false
     */
    bool moveTo(const Position& position, const bool keep_removed, std::vector<std::pair<Position, T>>& removed_region);

    /**
     * @brief 将地图视窗平移到新的中心位置（MapConcept 要求）
     * @param position 新的视窗中心位置（世界坐标）
     */
    void shiftWindowTo(const Position& position);

    /**
     * @brief 光线投射
     * @param origin 光线的终点位置
     * @param end 光线的终点位置
     * @param indices 返回光线经过的栅格索引
     * @return 如果光线终点超出地图范围，返回false；否则返回true
     */
    bool rayCast(const Position& origin,const Position& end, std::vector<Index>& indices) const;

    /**
     * @brief 设置地图的原点，即地图在世界坐标系中的位置
     * @param origin 地图的原点坐标
     */
    void setOrigin(const Position& origin);

    /**
     * @brief 获取满足某个条件的栅格索引
     * @param[out] indices 输出满足条件的栅格索引
     * @param[in] condition 条件函数，接受栅格数据类型T的引用，返回bool值
     */
    void selectCellsByCondition(std::vector<Index>& indices, std::function<bool(const T&)> condition) const;

    /**
     * @brief 获取地图中某个位置的索引
     * @param position 世界坐标系下的位置
     * @return 对应的栅格地图索引
     */
    Index getIndex(const Position& position) const;

    /**
     * @brief 获取地图中某个索引对应的栅格中心位置
     * @param index 栅格地图的索引
     * @return 对应的栅格中心位置
     */
    Position getPosition(const Index& index) const;

    /**
     * @brief 获取地图的尺寸
     * @return 地图的尺寸，单位为米
     */
    Length getLength() const;

    /**
     * @brief 获取栅格地图的大小
     * @return 栅格地图的大小，单位为栅格数
     */
    Size getSize() const;

    /**
     * @brief 获取地图分辨率
     * @return 地图分辨率，单位为米
     */
    double getResolution() const;

    /**
     * @brief 获取地图原点
     * @return 地图原点坐标
     */
    Position getOrigin() const;

    /**
     * @brief 地图边界框的最小索引
     * @return min_index
     */
    Index getMinIndex() const;


    /**
     * @brief 地图边界框的最大索引
     * @return max_index
     */
    Index getMaxIndex() const;

    /**
     * @brief 检查某个位置是否在地图范围内
     * @param position 世界坐标系下的位置
     * @return 如果位置在地图范围内返回true，否则返回false
     */
    bool isInside(const Position& position) const;

    /**
     * @brief 检查某个位置是否在地图范围内
     * @param index 栅格地图的索引
     * @return 如果位置在地图范围内返回true，否则返回false
     */
    bool isInside(const Index& index) const;


    /// @brief 迭代器接口
    using iterator = GridMapIterator<T>;
    using const_iterator = GridMapIterator<const T>;

    iterator begin() { return iterator(this, false); }
    iterator end() { return iterator(this, true); }
    const_iterator begin() const { return const_iterator(this, false); }
    const_iterator end() const { return const_iterator(this, true); }

private:
    std::vector<T> data_; // 存储所有栅格数据
    Length length_;       // 栅格地图的长度，单位为米
    Index start_index_;   // 地图数据的起始索引
    Size size_;           // 栅格地图的尺寸，一定为奇数，以保证原点在中心位置
    double resolution_;   // 栅格地图的分辨率，单位为米
    Position origin_;     // 当前栅格地图的原点，用于定义世界坐标系下地图的位置

    double inv_resolution_;
    Size half_size_;

};

}

#include "grid_map_impl.hpp"

FINENAV_REGISTER_TEMPLATE_ALGO_CONFIG(finenav::GridMap, grid_map)