// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include <shared_mutex>
#include "finenav_core/concepts.hpp"
namespace finenav {
/**
 * @brief LockedMapRO — 持有共享读锁的地图 RAII 视图。
 *
 * 构造时自动加读锁，析构时自动释放，保证 plan() 调用期间地图不被写入。
 */
template <MapConcept MapT>
class LockedMapRO {
public:
    LockedMapRO(const MapT& map, std::shared_mutex& mutex)
        : map_(map), lock_(mutex) {}
    const MapT* operator->() const { return &map_; }
    const MapT& get() const { return map_; }
private:
    const MapT& map_;
    std::shared_lock<std::shared_mutex> lock_;
};
/**
 * @brief LockedMapRW — 持有独占写锁的地图 RAII 视图。
 *
 * 构造时自动加写锁，析构时自动释放，用于主动且即时地修改地图。
 */
template <MapConcept MapT>
class LockedMapRW {
public:
    LockedMapRW(MapT& map, std::shared_mutex& mutex)
        : map_(map), lock_(mutex) {}
    MapT* operator->() { return &map_; }
    MapT& get() { return map_; }
private:
    MapT& map_;
    std::unique_lock<std::shared_mutex> lock_;
};
} // namespace finenav
