/**
 * @file ldlidar_datatype.h
 * @author LDRobot (support@ldrobot.com)
 * @brief LiDAR data types, enumerations and point-cloud structures.
 * @version 0.1
 * @date 2021-11-09
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _LDLIDAR_POINT_DATA_H_
#define _LDLIDAR_POINT_DATA_H_

#include <stdint.h>
#include <iostream>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/** @brief Converts an angle in degrees to radians. */
#define ANGLE_TO_RADIAN(angle) ((angle) * (M_PI / 180.0))
/** @brief Converts an angle in radians to degrees. */
#define RADIAN_TO_ANGLED(angle) ((angle) * (180.0 / M_PI))

namespace ldlidar {

/**
 * @enum LDType
 * @brief Supported LDRobot LiDAR product models.
 */
enum class LDType
{
  NO_VERSION, ///< Unspecified / not set
  LD_06,      ///< LDLidar LD-06
  LD_19,      ///< LDLidar LD-19
  STL_06P,    ///< LDLidar STL-06P
  STL_26,     ///< LDLidar STL-26
  STL_27L,    ///< LDLidar STL-27L
};

/**
 * @enum LidarStatus
 * @brief Operational status codes returned by driver API calls.
 */
enum class LidarStatus
{
  NORMAL,        ///< Lidar operating normally; point-cloud data is available
  ERROR,         ///< Lidar reported an internal hardware error
  DATA_TIME_OUT, ///< No new point-cloud packet was received within the timeout window
  DATA_WAIT,     ///< Waiting for the next complete point-cloud packet
  STOP,          ///< Driver has not been started yet
};

/**
 * @struct PointData
 * @brief A single measurement point expressed in both polar and Cartesian coordinates.
 */
struct PointData
{
  float    angle;     ///< Azimuth angle in the range [0, 360) degrees
  uint16_t distance;  ///< Radial distance in millimetres
  uint8_t  intensity; ///< Reflected signal intensity in the range [0, 255]
  uint64_t stamp;     ///< Host-system timestamp when the point was captured (nanoseconds)
  double   x;         ///< Cartesian X coordinate in metres (computed from polar data)
  double   y;         ///< Cartesian Y coordinate in metres (computed from polar data)

  /**
   * @brief Constructs a PointData with all fields explicitly set.
   * @param angle     Azimuth angle in degrees.
   * @param distance  Radial distance in millimetres.
   * @param intensity Reflected signal intensity.
   * @param stamp     Capture timestamp in nanoseconds (default 0).
   * @param x         Cartesian X in metres (default 0).
   * @param y         Cartesian Y in metres (default 0).
   */
  PointData(
    float angle, uint16_t distance, uint8_t intensity,
    uint64_t stamp = 0, double x = 0, double y = 0)
  {
    this->angle     = angle;
    this->distance  = distance;
    this->intensity = intensity;
    this->stamp     = stamp;
    this->x         = x;
    this->y         = y;
  }

  /** @brief Default constructor — all fields are left uninitialised. */
  PointData() {}
};

/** @brief Ordered collection of 2-D measurement points representing one full scan. */
typedef std::vector<PointData> Points2D;

/**
 * @struct LaserScan
 * @brief A complete 360° laser scan together with its capture timestamp.
 */
struct LaserScan
{
  uint64_t stamp;  ///< Host-system timestamp of the first point in the scan (nanoseconds)
  Points2D points; ///< Ordered array of measurement points for one full rotation

  /** @brief Copy-assignment operator. */
  LaserScan & operator=(const LaserScan & data)
  {
    this->stamp  = data.stamp;
    this->points = data.points;
    return *this;
  }
};

} // namespace ldlidar

#endif  // _POINT_DATA_H_
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/
