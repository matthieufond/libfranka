#pragma once
#include <fmt/core.h>
#include <iostream>
#include <sstream>
#include <tracy/Tracy.hpp>

inline double epoch_time_sec() {
  return std::chrono::steady_clock::now().time_since_epoch().count() / 1e9;
}

#define LOG_INFO(str, ...) logMessage(__FILE__, __LINE__, "INFO", str, ##__VA_ARGS__)
template <typename... T>
void logMessage(const char* filename,
                int line,
                const char* level,
                const std::string& format_str,
                T&&... args) {
  std::cout << fmt::format("[{}][{}]{}:{}: ",
                          epoch_time_sec(),
                          level,
                          filename,
                           line)
            << fmt::format(format_str, std::forward<T>(args)...) << std::endl;
}

#define LOG_INFO_ARRAY(prefix_str, array) logArray(__FILE__, __LINE__, "INFO", prefix_str, array)
inline void logArray(const char* filename,
              int line,
              const char* level,
              const std::string& name,
              const std::array<double, 7>& arr) {
  std::string str = name + ": ";
  for (size_t i = 0; i < arr.size(); i++) {
    str += std::to_string(i) + ":" + std::to_string(arr[i]) + ", ";
  }
  logMessage(filename, line, level, str);
}

#define CAT(a,b) a##b

#define TRACY_MESSAGE(var_name, msg, ...) \
  std::stringstream CAT(var_name,SS); \
  CAT(var_name,SS) << fmt::format("[{}]", epoch_time_sec()) << fmt::format(msg, ##__VA_ARGS__); \
  std::string var_name = CAT(var_name,SS).str(); \
  TracyMessage(var_name.c_str(), var_name.size());
