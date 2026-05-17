/**
 * @file ldlidar_driver.h
 * @author LDRobot (support@ldrobot.com)
 * @brief Top-level driver API for LDRobot LiDAR devices over serial communication.
 * @version 0.1
 * @date 2021-05-12
 *
 * @copyright Copyright (c) 2022  SHENZHEN LDROBOT CO., LTD. All rights
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
#ifndef __LDLIDAR_NODE_H__
#define __LDLIDAR_NODE_H__

#include <chrono>
#include <functional>

#include "serial_interface_linux.h"
#include "lipkg.h"
#include "log_module.h"

namespace ldlidar {

/**
 * @enum CommunicationModeTypeDef
 * @brief Supported physical communication channels.
 */
typedef enum CommunicationMode
{
  COMM_NO_NULL,      ///< Unspecified / not set
  COMM_SERIAL_MODE,  ///< UART / serial communication
} CommunicationModeTypeDef;

/**
 * @class LDLidarDriver
 * @brief High-level driver that owns the serial link and packet parser.
 *
 * Typical usage:
 * -# Construct an instance.
 * -# Call Start() to open the serial port and start the receive thread.
 * -# Call WaitLidarCommConnect() to confirm the LiDAR is responding.
 * -# Poll GetLaserScanData() from your processing thread.
 * -# Call Stop() to cleanly shut down.
 */
class LDLidarDriver
{
public:
  LDLidarDriver();
  ~LDLidarDriver();

  /**
   * @brief Opens the serial port and starts the LiDAR communication.
   * @param product_name      LDRobot product model (@see ldlidar::LDType).
   * @param serial_port_name  Path to the serial device (e.g. @c "/dev/ttyUSB0").
   * @param serial_baudrate   Baud rate in bits per second (default 115200).
   * @param comm_mode         Communication channel; only COMM_SERIAL_MODE is supported.
   * @return @c true on success; @c false if the port could not be opened.
   */
  bool Start(
    LDType product_name,
    std::string serial_port_name,
    uint32_t serial_baudrate = 115200,
    CommunicationModeTypeDef comm_mode = COMM_SERIAL_MODE);

  /**
   * @brief Stops the LiDAR communication and closes the serial port.
   * @return @c true on success; @c false if the driver was not started.
   */
  bool Stop(void);

  /**
   * @brief Blocks until the LiDAR has sent its first valid packet or the timeout expires.
   * @param timeout  Maximum wait time in milliseconds.
   * @return @c true if communication was established within the timeout; @c false otherwise.
   */
  bool WaitLidarCommConnect(int64_t timeout);

  /**
   * @brief Retrieves the latest complete 360° point cloud.
   * @param[out] dst      Destination vector populated with the scan points.
   * @param[in]  timeout  Maximum wait time for a new frame in milliseconds (default 1000).
   * @return A ldlidar::LidarStatus value:
   *   - @c LidarStatus::NORMAL       — new data was returned in @p dst
   *   - @c LidarStatus::DATA_WAIT    — no new frame yet; @p dst is unchanged
   *   - @c LidarStatus::DATA_TIME_OUT — timeout elapsed with no data
   *   - @c LidarStatus::ERROR        — the LiDAR reported a hardware error
   */
  LidarStatus GetLaserScanData(Points2D & dst, int64_t timeout = 1000);

  /**
   * @brief Returns the current LiDAR rotation speed.
   * @param[out] spin_hz  Rotation frequency in Hz.
   * @return @c true on success; @c false if the driver has not been started.
   */
  bool GetLidarScanFreq(double & spin_hz);

  /**
   * @brief Registers a function that provides nanosecond-resolution host timestamps.
   *
   * The registered function is called once per received packet to stamp the data.
   * If not registered, packets will carry a zero timestamp.
   *
   * @param get_timestamp_handle  Callable with signature @c uint64_t(void).
   */
  void RegisterGetTimestampFunctional(std::function<uint64_t(void)> get_timestamp_handle);

  /**
   * @brief Enables or disables the spatial noise/near-range filter.
   * @param is_enable  @c true to enable filtering; @c false to pass raw points through.
   */
  void EnableFilterAlgorithnmProcess(bool is_enable);

private:
  std::string sdk_version_number_;                           ///< SDK version string (for diagnostics)
  bool is_start_flag_;                                       ///< True between a successful Start() and Stop()
  std::function<uint64_t(void)> register_get_timestamp_handle_; ///< Stored timestamp callback
  LiPkg * comm_pkg_;                                         ///< Packet parser / assembler
  SerialInterfaceLinux * comm_serial_;                       ///< Serial port interface
  std::chrono::_V2::steady_clock::time_point last_pubdata_times_; ///< Time of the last successful data retrieval
};

} // namespace ldlidar

#endif // __LDLIDAR_DRIVER_H__
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/
