#pragma once

#include <memory>
#include <spdlog/spdlog.h>

struct log {
public:
  static void init();

  inline static std::shared_ptr<spdlog::logger> &get_logger() {
    return core_logger;
  }

private:
  static std::shared_ptr<spdlog::logger> core_logger;
};

// log macros
#define TINY_CAD_TRACE(...) ::log::get_logger()->trace(__VA_ARGS__)
#define TINY_CAD_INFO(...) ::log::get_logger()->info(__VA_ARGS__)
#define TINY_CAD_WARN(...) ::log::get_logger()->warn(__VA_ARGS__)
#define TINY_CAD_ERROR(...) ::log::get_logger()->error(__VA_ARGS__)
#define TINY_CAD_CRITICAL(...) ::log::get_logger()->critical(__VA_ARGS__)
