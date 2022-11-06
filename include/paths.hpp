#pragma once

#include <filesystem>

namespace paths {

void generate_first_stage(float *data, int resolution, int size,
                          std::filesystem::path path);

}
