#include <log.h>
#include <spdlog/sinks/stdout_color_sinks.h>

std::shared_ptr<spdlog::logger> log::core_logger;

void log::init() {
  spdlog::set_pattern("%^[%T] %n: %v%$");
  core_logger = spdlog::stdout_color_mt("TinyCAD");
  core_logger->set_level(spdlog::level::trace);
  TINY_CAD_INFO("Initialized log!");
}
