#ifndef SWARM_PLANNER_GEO_UTILS_H_
#define SWARM_PLANNER_GEO_UTILS_H_

#include "swarm_planner/planner_types.h"

namespace swarm_planner {
namespace geo {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kEarthRadiusM = 6378137.0;

struct GpsOrigin
{
    double lat_rad{0.0};
    double lon_rad{0.0};
    double alt_m{0.0};

    void set(double lat_deg, double lon_deg, double alt_msl)
    {
        lat_rad = lat_deg * kDegToRad;
        lon_rad = lon_deg * kDegToRad;
        alt_m = alt_msl;
    }
};

// 平面切平面近似 LLA→NED，NED z 轴向下（高度取反）。
// 适用范围：编队工作范围相对地球半径足够小（< 10 km）。
Vector3 lla_to_ned(double lat_deg, double lon_deg, double alt_m, const GpsOrigin& origin);

}  // namespace geo
}  // namespace swarm_planner

#endif  // SWARM_PLANNER_GEO_UTILS_H_
