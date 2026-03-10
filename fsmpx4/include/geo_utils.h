#ifndef GEO_UTILS_H_
#define GEO_UTILS_H_

#include <cmath>
#include "types.h"

namespace fsmpx4 {
namespace geo {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kEarthRadiusM = 6378137.0;

/// GPS origin for LLA → NED conversion.
struct GpsOrigin
{
    double lat_rad{0.0};
    double lon_rad{0.0};
    double alt_m{0.0};
    bool initialized{false};

    void set(double lat_deg, double lon_deg, double alt_msl)
    {
        lat_rad = lat_deg * kDegToRad;
        lon_rad = lon_deg * kDegToRad;
        alt_m = alt_msl;
        initialized = true;
    }
};

/// Pure function: LLA (degrees, meters) → NED (meters) relative to origin.
inline types::Vector3 lla_to_ned(double lat_deg, double lon_deg, double alt_m,
                                 const GpsOrigin& origin)
{
    const double lat_rad = lat_deg * kDegToRad;
    const double lon_rad = lon_deg * kDegToRad;
    return {
        (lat_rad - origin.lat_rad) * kEarthRadiusM,
        (lon_rad - origin.lon_rad) * kEarthRadiusM * std::cos(origin.lat_rad),
        origin.alt_m - alt_m
    };
}

}  // namespace geo
}  // namespace fsmpx4

#endif  // GEO_UTILS_H_
