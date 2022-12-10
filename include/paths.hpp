#pragma once

#include <filesystem>
#include <vector>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

namespace paths {

void generate_first_stage(float *data, int resolution, int size,
                          std::filesystem::path path);

void generate_second_stage(std::filesystem::path path);

void generate_third_stage(std::filesystem::path path);
} // namespace paths
