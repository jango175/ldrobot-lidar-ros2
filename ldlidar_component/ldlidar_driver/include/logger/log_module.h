/**
 * @file log_module.h
 * @author LDRobot (support@ldrobot.com)
 * @brief Lightweight logging framework for the LDRobot LiDAR SDK.
 *
 * Provides a singleton LogModule that formats and routes log messages to a
 * pluggable ILogRealization backend.  On Linux the default backend (LogPrint)
 * writes to stdout when ENABLE_CONSOLE_LOG_DIS is defined.
 *
 * Convenience macros (LD_LOG_DEBUG / INFO / WARN / ERROR and their LDS_* no-location
 * variants) are the intended public API; direct use of LogModule is not required.
 *
 * @version 0.1
 * @date 2022-08-10
 *
 * @copyright Copyright (c) 2022  SHENZHEN LDROBOT CO., LTD. All rights reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef  __LOGMODULE_H_
#define  __LOGMODULE_H_


#define LINUX

/** @brief Compile-time switch: define to enable all log output. */
#define ENABLE_LOG_DIS_OUTPUT

/** @brief Compile-time switch: routes log output to the console (stdout). */
#define ENABLE_CONSOLE_LOG_DIS

//#define ENABLE_LOG_WRITE_TO_FILE

/** @brief Path to the optional log file used when ENABLE_LOG_WRITE_TO_FILE is defined. */
#define LOGFILEPATH "./ldlidar-driver.log"

#include <stdio.h>
#include <string>
#include <chrono>
#include <stdlib.h>

#ifndef LINUX
#include <windows.h>
#else
//#include <pthread.h>
#include <stdarg.h>
#define printf_s(fileptr, str)  (fprintf(fileptr, "%s", str))
#define __in
#endif


/**
 * @struct LogVersion
 * @brief Associates a numeric version with a human-readable description string.
 */
struct LogVersion
{
  int n_version;           ///< Numeric version identifier
  std::string str_descruble; ///< Human-readable description of this version
};


/**
 * @class ILogRealization
 * @brief Abstract backend interface for log output.
 *
 * Derive from this class and implement Initializion() and LogPrintInf() to
 * redirect log output to a custom sink (file, socket, etc.).  Pass an instance
 * to LogModule::GetInstance() to swap the active backend.
 */
class ILogRealization
{
public:
  virtual ~ILogRealization() {}

  /**
   * @brief Optional one-time initialisation for the backend (e.g. open a file).
   * @param path  Backend-specific path string; may be @c NULL.
   */
  virtual void Initializion(const char * path = NULL) = 0;

  /**
   * @brief Writes a fully-formatted log line to the backend.
   * @param str  Null-terminated log string including level, timestamp, and message.
   */
  virtual void LogPrintInf(const char * str) = 0;

  /** @brief Destroys this instance by dispatching to the virtual free(ILogRealization*) overload. */
  void free();

private:
  /** @brief Subclasses must implement this to delete themselves safely (see ILOGFREE macro). */
  virtual void free(ILogRealization * plogger) = 0;
};


/**
 * @brief Helper macro that generates the polymorphic free() implementation for a concrete backend.
 *
 * Usage: place @c ILOGFREE(MyBackendClass) inside the class body.
 */
#define  ILOGFREE(LogRealizationClass)  virtual void free(ILogRealization * plogger) \
  { \
    LogRealizationClass * prealization = static_cast < LogRealizationClass * > (plogger); \
    if (prealization != NULL) {delete prealization;} \
  }

/**
 * @class LogPrint
 * @brief Console (stdout) log backend — the default backend on Linux.
 *
 * Writes each formatted log line to stdout when ENABLE_CONSOLE_LOG_DIS is defined.
 * Optionally appends to the file at LOGFILEPATH when ENABLE_LOG_WRITE_TO_FILE is defined.
 */
class LogPrint : public ILogRealization
{
public:
  virtual void Initializion(const char * path = NULL);
  virtual void free(ILogRealization * plogger);
  virtual void LogPrintInf(const char * str);
};

#ifndef LINUX
/**
 * @class LogOutputString
 * @brief Windows debug-output backend; routes log lines to OutputDebugString().
 */
class LogOutputString : public ILogRealization
{
public:
  virtual void Initializion(const char * path = NULL) { return; }

  virtual void LogPrintInf(const char * str)
  {
    OutputDebugString((LPCTSTR)str);
    OutputDebugString("\r\n");
  }

  ILOGFREE(LogOutputString)
};
#endif


/**
 * @class LogModule
 * @brief Singleton log dispatcher that formats messages and forwards them to an ILogRealization backend.
 *
 * Use the LD_LOG_* / LDS_LOG_* macros rather than calling this class directly.
 * The singleton is created on first use; the backend defaults to LogPrint on Linux.
 */
class LogModule
{
public:
  /**
   * @enum LogLevel
   * @brief Severity levels for log messages.
   */
  enum LogLevel
  {
    DEBUG_LEVEL,   ///< Verbose debug output including file, function, and line information
    WARNING_LEVEL, ///< Non-fatal conditions that may warrant attention
    ERROR_LEVEL,   ///< Recoverable errors
    INFO_LEVEL     ///< General informational messages
  };

  /**
   * @struct LOGMODULE_INFO
   * @brief Context captured at the macro call site and attached to each log entry.
   */
  struct LOGMODULE_INFO
  {
    LogLevel    loglevel;       ///< Severity level of the pending message
    std::string str_filename;   ///< Source file name (__FILE__)
    std::string str_funcname;   ///< Enclosing function name (__FUNCTION__)
    int         n_linenumber;   ///< Source line number (__LINE__)
  } logInfo_; ///< Call-site context for the next LogPrintInf() call

  ILogRealization * p_realization_; ///< Active output backend; never null after construction

public:
  /**
   * @brief Returns the singleton, setting the call-site context and optional new backend.
   * @param filename  Source file path (pass @c __FILE__).
   * @param funcname  Enclosing function name (pass @c __FUNCTION__).
   * @param lineno    Source line number (pass @c __LINE__).
   * @param level     Severity level for the next message.
   * @param plog      Optional replacement backend; ownership is transferred on success.
   * @return Pointer to the singleton LogModule instance.
   */
  static LogModule * GetInstance(
    __in const char * filename, __in const char * funcname,
    __in int lineno, LogLevel level, ILogRealization * plog = NULL);

  /**
   * @brief Returns the singleton, setting only the log level (no call-site context).
   * @param level  Severity level for the next message.
   * @param plog   Optional replacement backend; ownership is transferred on success.
   * @return Pointer to the singleton LogModule instance.
   */
  static LogModule * GetInstance(LogLevel level, ILogRealization * plog = NULL);

  /**
   * @brief Formats and emits a log line that includes the file/function/line context.
   * @param format  printf-style format string.
   */
  void LogPrintInf(const char * format, ...);

  /**
   * @brief Formats and emits a log line without file/function/line context.
   * @param format  printf-style format string.
   */
  void LogPrintNoLocationInf(const char * format, ...);

private:
  LogModule();
  ~LogModule();

  void InitLock();
  void RealseLock();
  void Lock();
  void UnLock();

  std::string GetCurrentTime();
  uint64_t    GetCurrentLocalTimeStamp();
  std::string GetFormatValue(std::string str_value);
  std::string GetFormatValue(int n_value);
  std::string GetLevelValue(int level);

  static LogModule * s_plog_module_; ///< The singleton instance

#ifndef LINUX
  CRITICAL_SECTION mutex_lock_;
#else
  pthread_mutex_t mutex_lock_; ///< Mutex protecting concurrent LogPrintInf() calls
#endif
};

/** @name Location-aware logging macros
 *  Include the source file, function, and line number in the log entry.
 *  @{ */
#define  LOG(level, format, ...)   LogModule::GetInstance( \
    __FILE__, __FUNCTION__, __LINE__, \
    level)->LogPrintInf(format, __VA_ARGS__);
#ifdef ENABLE_LOG_DIS_OUTPUT
#define  LD_LOG_DEBUG(format, ...)   LOG(LogModule::DEBUG_LEVEL, format, __VA_ARGS__)   ///< Debug log with location
#define  LD_LOG_INFO(format, ...)    LOG(LogModule::INFO_LEVEL, format, __VA_ARGS__)    ///< Info log with location
#define  LD_LOG_WARN(format, ...)    LOG(LogModule::WARNING_LEVEL, format, __VA_ARGS__) ///< Warning log with location
#define  LD_LOG_ERROR(format, ...)   LOG(LogModule::ERROR_LEVEL, format, __VA_ARGS__)   ///< Error log with location
#else
#define  LD_LOG_DEBUG(format, ...)   do {} while(0)
#define  LD_LOG_INFO(format, ...)    do {} while(0)
#define  LD_LOG_WARN(format, ...)    do {} while(0)
#define  LD_LOG_ERROR(format, ...)   do {} while(0)
#endif
/** @} */

/** @name Location-free logging macros
 *  Emit a log entry without file/function/line context.
 *  @{ */
#ifdef ENABLE_LOG_DIS_OUTPUT
#define  LOG_NO_DESCRI(level, format, ...)   LogModule::GetInstance(level)->LogPrintNoLocationInf( \
    format, __VA_ARGS__);
#define  LDS_LOG_DEBUG(format, ...)   LOG_NO_DESCRI(LogModule::DEBUG_LEVEL, format, __VA_ARGS__)   ///< Debug log without location
#define  LDS_LOG_INFO(format, ...)    LOG_NO_DESCRI(LogModule::INFO_LEVEL, format, __VA_ARGS__)    ///< Info log without location
#define  LDS_LOG_WARN(format, ...)    LOG_NO_DESCRI(LogModule::WARNING_LEVEL, format, __VA_ARGS__) ///< Warning log without location
#define  LDS_LOG_ERROR(format, ...)   LOG_NO_DESCRI(LogModule::ERROR_LEVEL, format, __VA_ARGS__)   ///< Error log without location
#else
#define  LDS_LOG_DEBUG(format, ...)   do {} while(0)
#define  LDS_LOG_INFO(format, ...)    do {} while(0)
#define  LDS_LOG_WARN(format, ...)    do {} while(0)
#define  LDS_LOG_ERROR(format, ...)   do {} while(0)
#endif
/** @} */

#endif//__LDLIDAR_LOGGER_H__
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/
