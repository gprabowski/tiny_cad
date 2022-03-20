#pragma once

#include <filesystem>
#include <glad/glad.h>
#include <string>

namespace shader {
std::string
read_shader_file(const std::filesystem::path::value_type *shader_file);

GLuint LoadProgram(const std::string &name);

} // namespace shader
