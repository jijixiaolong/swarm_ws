#include "swarm_planner/geo_utils.h"

#include <cmath>

namespace swarm_planner {
namespace geo {

Vector3 lla_to_ned(
    const double lat_deg,
    const double lon_deg,
    const double alt_m,
    const GpsOrigin& origin)
{
    const double lat_rad = lat_deg * kDegToRad;
    const double lon_rad = lon_deg * kDegToRad;
    return {
        (lat_rad - origin.lat_rad) * kEarthRadiusM,
        (lon_rad - origin.lon_rad) * kEarthRadiusM * std::cos(origin.lat_rad),
        origin.alt_m - alt_m};
}

}  // namespace geo
}  // namespace swarm_planner
