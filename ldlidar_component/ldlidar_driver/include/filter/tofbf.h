/**
 * @file tofbf.h
 * @author LDRobot (support@ldrobot.com)
 * @brief LiDAR near-range and noise filtering algorithms.
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __TOFBF_H_
#define __TOFBF_H_

#include <stdint.h>

#include <vector>
#include <math.h>

#include <algorithm>
#include <iostream>

#include "ldlidar_datatype.h"

namespace ldlidar {

/**
 * @enum FilterType
 * @brief Selects which spatial filter pass is applied to incoming point data.
 */
enum class FilterType
{
  NO_FILTER,    ///< No filtering; all points are passed through unchanged
  NEAR_FILTER,  ///< Removes spurious near-range returns caused by stray reflections
  NOISE_FILTER  ///< Removes isolated noise points that have no neighbouring returns
};

/**
 * @class Tofbf
 * @brief Applies near-range and noise rejection filters to a LiDAR point cloud.
 *
 * The filter type is selected automatically at construction time based on the
 * LiDAR model and the reported rotation speed.  Call Filter() on each frame to
 * obtain a cleaned point vector.
 *
 * The class is non-copyable; construct one instance per driver instance.
 */
class Tofbf
{
public:
  /**
   * @brief Constructs a filter tuned for the given model and rotation speed.
   * @param speed  Current LiDAR rotation speed in degrees per second.
   * @param type   LDRobot product model (@see ldlidar::LDType).
   */
  Tofbf(int speed, LDType type);

  ~Tofbf();

  /**
   * @brief Filters a raw point cloud and returns the cleaned result.
   * @param tmp  Input vector of raw measurement points for one full rotation.
   * @return     Vector containing only the points that passed the selected filter.
   */
  std::vector<PointData> Filter(const std::vector<PointData> & tmp) const;

private:
  FilterType filter_type_;  ///< Active filter mode selected at construction
  int intensity_low_;       ///< Intensity threshold below which a point is considered weak
  int intensity_single_;    ///< Minimum intensity required for a lone (isolated) point to survive
  int scan_frequency_;      ///< Nominal scan frequency used for neighbourhood calculations (Hz)
  double curr_speed_;       ///< Actual rotation speed passed to the constructor (deg/s)

  Tofbf() = delete;
  Tofbf(const Tofbf &) = delete;
  Tofbf & operator=(const Tofbf &) = delete;

  /**
   * @brief Removes near-range returns that are likely caused by lens or housing reflections.
   * @param tmp  Input point cloud.
   * @return     Filtered point cloud.
   */
  std::vector<PointData> NearFilter(const std::vector<PointData> & tmp) const;

  /**
   * @brief Removes isolated points that have no neighbours within a distance threshold.
   * @param tmp  Input point cloud.
   * @return     Filtered point cloud.
   */
  std::vector<PointData> NoiseFilter(const std::vector<PointData> & tmp) const;
};

} // namespace ldlidar

#endif  //__TOFBF_H_

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/
