#pragma once
#include <fmt/core.h>
#include <iostream>

#define LOG_INFO(str, ...) logMessage(__FILE__, __LINE__, "INFO", str, ##__VA_ARGS__)
template <typename... T>
void logMessage(const char* filename,
                int line,
                const char* level,
                const std::string& format_str,
                T&&... args) {
  std::cout << fmt::format("[{}][{}]{}:{}: ", level,
                           std::chrono::steady_clock::now().time_since_epoch().count(), filename,
                           line)
            << fmt::format(format_str, std::forward<T>(args)...) << std::endl;
}

#define LOG_INFO_ARRAY(prefix_str, array) logArray(__FILE__, __LINE__, "INFO", prefix_str, array)
void logArray(const char* filename,
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
