/**
 * @file serial_interface_linux.h
 * @author LDRobot (support@ldrobot.com)
 * @brief Linux serial port driver for LDRobot LiDAR communication.
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

#ifndef __LINUX_SERIAL_PORT_H__
#define __LINUX_SERIAL_PORT_H__

#include <inttypes.h>
#include <errno.h>
#include <fcntl.h>
#include <memory.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
namespace asmtermios {
#include <linux/termios.h>
}
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ldlidar {

/**
 * @class SerialInterfaceLinux
 * @brief Non-blocking serial port driver that delivers incoming bytes via a callback.
 *
 * Opens a serial device, configures it with @c termios2 (supporting arbitrary baud rates),
 * and spawns a background receive thread.  Incoming data is delivered synchronously from
 * that thread through the callback registered with SetReadCallback().
 */
class SerialInterfaceLinux
{
public:
  SerialInterfaceLinux();
  ~SerialInterfaceLinux();

  /**
   * @brief Opens the serial port and starts the receive thread.
   * @param port_name    Path to the serial device (e.g. @c "/dev/ttyUSB0").
   * @param com_baudrate Desired baud rate in bits per second.
   * @return @c true on success; @c false if the port could not be opened or configured.
   */
  bool Open(std::string & port_name, uint32_t com_baudrate);

  /**
   * @brief Closes the serial port and joins the receive thread.
   * @return @c true on success; @c false if the port was already closed.
   */
  bool Close();

  /**
   * @brief Reads available bytes from the serial port with a 100 ms timeout.
   * @param[out] rx_buf     Buffer to receive the incoming bytes.
   * @param[in]  rx_buf_len Capacity of @p rx_buf in bytes.
   * @param[out] rx_len     Number of bytes actually read; unchanged on failure.
   * @return @c true if at least one byte was read; @c false on timeout or error.
   */
  bool ReadFromIO(uint8_t * rx_buf, uint32_t rx_buf_len, uint32_t * rx_len);

  /**
   * @brief Registers the callback that is invoked for every received data chunk.
   * @param callback Function called from the receive thread with a pointer to the
   *                 received bytes and their count.
   */
  void SetReadCallback(std::function<void(const char *, size_t length)> callback);

  /**
   * @brief Returns whether the serial port is currently open.
   * @return @c true if Open() has succeeded and Close() has not been called since.
   */
  bool IsOpened();

private:
  std::thread * rx_thread_;                                          ///< Background receive thread
  long long rx_count_;                                               ///< Total bytes received since Open()
  int32_t com_handle_;                                               ///< File descriptor for the serial device
  uint32_t com_baudrate_;                                            ///< Configured baud rate (bits/s)
  std::atomic<bool> is_cmd_opened_;                                  ///< Set to true while the port is open
  std::atomic<bool> rx_thread_exit_flag_;                            ///< Set to true to signal the receive thread to exit
  std::function<void(const char *, size_t length)> read_callback_;   ///< User-supplied data callback

  /**
   * @brief Receive thread entry point; loops calling ReadFromIO() until signalled to stop.
   * @param param Pointer to the owning SerialInterfaceLinux instance.
   */
  static void RxThreadProc(void * param);
};

} // namespace ldlidar

#endif  //__LINUX_SERIAL_PORT_H__

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/
