#pragma once

#include <array>
#include <cstdint>

namespace ecs {
using EntityType = uint64_t;
using ComponentType = uint64_t;
using component_bitset = uint64_t;

enum ct : component_bitset {
  PARAMETRIC_COM = 1 << 0,
  TRANSFORMATION_COM = 1 << 1,
  OGL_COM = 1u << 2,
  TORUS_COM = 1u << 3,
  TAG_FIGURE = 1u << 4,
  TAG_CURSOR = 1u << 5,
  TAG_POINT = 1u << 6,
  OTHER = 1u << 31,
};

constexpr std::array<ct, 8> all_components{
    PARAMETRIC_COM, TRANSFORMATION_COM, OGL_COM,   TORUS_COM,
    TAG_FIGURE,     TAG_CURSOR,         TAG_POINT, OTHER};
} // namespace ecs
