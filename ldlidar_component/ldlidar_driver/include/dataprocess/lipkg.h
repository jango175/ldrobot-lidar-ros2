/**
 * @file lipkg.h
 * @author LDRobot (support@ldrobot.com)
 * @brief LiDAR serial-packet parser and point-cloud assembler.
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

#ifndef __LIPKG_H
#define __LIPKG_H


#include <chrono>
#include <functional>
#include <mutex>

#include <string.h>

#include "ldlidar_datatype.h"
#include "tofbf.h"

namespace ldlidar {

/** @name Wire-protocol constants
 *  @{ */
enum
{
  PKG_HEADER     = 0x54, ///< Fixed header byte that marks the start of every LiDAR packet
  PKG_VER_LEN    = 0x2C, ///< Version/length byte expected in every valid packet header
  POINT_PER_PACK = 12,   ///< Number of measurement points carried in one packet
};
/** @} */

/**
 * @struct LidarPointStructDef
 * @brief A single raw measurement point as encoded on the wire (packed, no padding).
 */
typedef struct __attribute__((packed))
{
  uint16_t distance;  ///< Radial distance in millimetres
  uint8_t intensity;  ///< Reflected signal intensity [0, 255]
} LidarPointStructDef;

/**
 * @struct LiDARFrameTypeDef
 * @brief A complete LiDAR serial packet as received from the device (packed, no padding).
 *
 * Contains the spin speed, start/end angles for interpolation, twelve measurement
 * points, a packet timestamp, and a CRC-8 checksum.
 */
typedef struct __attribute__((packed))
{
  uint8_t  header;                           ///< Must equal PKG_HEADER (0x54)
  uint8_t  ver_len;                          ///< Must equal PKG_VER_LEN (0x2C)
  uint16_t speed;                            ///< Rotation speed in degrees per second
  uint16_t start_angle;                      ///< Angle of the first point × 100 (hundredths of a degree)
  LidarPointStructDef point[POINT_PER_PACK]; ///< Array of raw measurement points
  uint16_t end_angle;                        ///< Angle of the last point × 100 (hundredths of a degree)
  uint16_t timestamp;                        ///< Device-internal packet timestamp (ms, wraps at 30 000)
  uint8_t  crc8;                             ///< CRC-8 checksum over all preceding bytes in the packet
} LiDARFrameTypeDef;

/**
 * @class LiPkg
 * @brief Parses the LiDAR binary protocol and assembles full-rotation point clouds.
 *
 * Feed raw serial bytes via CommReadCallback().  When a complete 360° scan has been
 * assembled, GetLaserScanData() will return it.  Optionally register a timestamp
 * provider with RegisterTimestampGetFunctional() and enable the spatial filter with
 * EnableFilter().
 */
class LiPkg
{
public:
  LiPkg();
  ~LiPkg();

  /**
   * @brief Sets the LiDAR product model, which controls angle interpolation and filter tuning.
   * @param type_number  One of the ldlidar::LDType enumerators.
   */
  void SetProductType(LDType type_number);

  /**
   * @brief Returns the current reported LiDAR rotation speed.
   * @return Spin frequency in Hz.
   */
  double GetSpeed(void);

  /**
   * @brief Receive callback — feed raw serial bytes from the driver into the parser.
   * @param byte  Pointer to the received byte buffer.
   * @param len   Number of bytes in the buffer.
   */
  void CommReadCallback(const char * byte, size_t len);

  /**
   * @brief Retrieves the latest complete scan when one is ready.
   * @param[out] out  Destination vector; populated only when a new frame is available.
   * @return @c true if a new frame was ready and @p out was populated; @c false otherwise.
   */
  bool GetLaserScanData(Points2D & out);

  /**
   * @brief Registers an external function used to stamp each received packet.
   * @param timestamp_handle  Callable returning a nanosecond-resolution host timestamp.
   */
  void RegisterTimestampGetFunctional(std::function<uint64_t(void)> timestamp_handle);

  /**
   * @brief Returns whether a valid communication packet was received after power-on.
   * @return @c true once the first well-formed packet has been parsed.
   */
  bool GetLidarPowerOnCommStatus(void);

  /**
   * @brief Enables or disables the spatial filter applied to each assembled frame.
   * @param is_enable  @c true to enable filtering; @c false to pass raw points through.
   */
  void EnableFilter(bool is_enable);

  /**
   * @brief Returns the current driver status (NORMAL, ERROR, DATA_TIME_OUT, etc.).
   * @return A ldlidar::LidarStatus enumerator.
   */
  LidarStatus GetLidarStatus(void);

  /**
   * @brief Resets all parser and frame-assembly state.
   *
   * Call this before restarting the driver to prevent stale state from corrupting
   * the first frame of the new session.
   */
  void ClearDataProcessStatus(void);

private:
  /** @brief Byte-level parser states for the two-byte packet header. */
  enum class ParserState { HEADER, VER_LEN, DATA };

  LDType   product_type_;            ///< Active LiDAR model, set by SetProductType()
  uint16_t timestamp_;               ///< Device timestamp from the most recently parsed packet
  double   speed_;                   ///< Rotation speed from packets (deg/s internally, Hz via GetSpeed())
  bool     is_frame_ready_;          ///< True when a full-rotation frame is waiting to be collected
  bool     is_poweron_comm_normal_;  ///< True after the first valid packet has been received
  bool     is_filter_;               ///< Whether spatial filtering is enabled
  LidarStatus lidarstatus_;          ///< Current driver status exposed through GetLidarStatus()
  int measure_point_frequence_;      ///< Expected points per rotation; used as assembly overflow guard
  std::function<uint64_t(void)> get_timestamp_; ///< User-supplied nanosecond timestamp provider
  uint64_t last_pkg_timestamp_;      ///< Timestamp of the previous packet, used to detect timeouts
  bool first_frame_;                 ///< True until the first complete frame has been assembled

  ParserState parser_state_;         ///< Current byte-level parser state
  uint16_t    parser_count_;         ///< Number of DATA bytes accumulated so far
  uint8_t     parser_tmp_[128];      ///< Staging buffer for the packet body during parsing
  static constexpr uint16_t PARSER_PKG_COUNT = sizeof(LiDARFrameTypeDef); ///< Expected packet body length

  LiDARFrameTypeDef pkg_;            ///< Last fully parsed wire packet
  Points2D frame_tmp_;               ///< Point accumulator for the current in-progress rotation
  Points2D laser_scan_data_;         ///< Latest complete frame, guarded by mutex_lock2_
  std::mutex mutex_lock1_;           ///< Protects is_frame_ready_ and frame_tmp_
  std::mutex mutex_lock2_;           ///< Protects laser_scan_data_

  /**
   * @brief Processes one byte through the header/body state machine.
   * @param byte  Incoming byte.
   * @return @c true when a complete, CRC-validated packet has been assembled into pkg_.
   */
  bool AnalysisOne(uint8_t byte);

  /**
   * @brief Feeds a buffer of raw bytes through AnalysisOne() one byte at a time.
   * @param data  Pointer to the byte buffer.
   * @param len   Number of bytes to process.
   * @return @c true if at least one complete packet was produced.
   */
  bool Parse(const uint8_t * data, long len);

  /**
   * @brief Converts a parsed packet into PointData entries and checks for a full rotation.
   * @return @c true when a complete 360° frame has been assembled.
   */
  bool AssemblePacket();

  /** @brief Marks the current frame as ready and copies it to laser_scan_data_. */
  void SetFrameReady(void);

  /**
   * @brief Atomically copies @p src into laser_scan_data_.
   * @param src  Assembled point cloud for the current rotation.
   */
  void SetLaserScanData(Points2D & src);

  /**
   * @brief Returns whether a new complete frame is waiting to be collected.
   * @return @c true if a frame is ready.
   */
  bool IsFrameReady(void);

  /** @brief Clears the frame-ready flag after the frame has been retrieved. */
  void ResetFrameReady(void);
};

} // namespace ldlidar

#endif  //__LIPKG_H

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/
