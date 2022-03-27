#pragma once

#include <array>
#include <cstdint>

namespace ecs {
using EntityType = uint64_t;
constexpr EntityType null_entity = static_cast<EntityType>(-1);
using ComponentType = uint64_t;
using component_bitset = uint64_t;

enum ct : component_bitset {
  PARAMETRIC_COM = 1 << 0,
  TRANSFORMATION_COM = 1 << 1,
  OGL_COM = 1ull << 2,
  TORUS_COM = 1ull << 3,
  TAG_FIGURE = 1ull << 4,
  TAG_CURSOR = 1ull << 5,
  TAG_POINT = 1ull << 6,
  TAG_SELECTED = 1ull << 7,
  TAG_BEZIERC = 1ull << 8,
  RELATIONSHIP = 1ull << 9,
  OTHER = 1ull << 63,
};

constexpr std::array<ct, 11> all_components{
    PARAMETRIC_COM, TRANSFORMATION_COM, OGL_COM,   TORUS_COM,
    TAG_FIGURE,     TAG_CURSOR,         TAG_POINT, TAG_SELECTED,
    TAG_BEZIERC,    RELATIONSHIP,       OTHER};
} // namespace ecs
