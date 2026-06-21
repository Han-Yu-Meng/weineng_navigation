#ifndef FINENAV_MAP_VIEW_DEF_HPP
#define FINENAV_MAP_VIEW_DEF_HPP

#include <Eigen/Core>

using Position3D = Eigen::Vector3d;
namespace nav2_mppi_controller {
    class IMapView {
    public:
        IMapView() =  default;
        virtual ~IMapView() = default;
        // 是否将未知区域视为可通行
        virtual bool isTrackingUnknown() const = 0;

        virtual bool considerFootprint() const = 0;

        virtual bool isCollision(
          float x, float y, float theta) const = 0;

        virtual float getRadius() const = 0;

        // 获取地图索引处的代价值（根据项目需要可改为返回 unsigned char 等）
        virtual int getCost(const Position3D& pos ) const = 0;

        virtual float costAtPose(float x, float y, float theta) const = 0;

        virtual std::string getBaseFrameID() const = 0;
    };
}

#endif
