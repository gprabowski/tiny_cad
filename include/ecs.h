#pragma once

#include <array>
#include <cstdint>

namespace ecs {
using EntityType = uint64_t;
constexpr EntityType null_entity = static_cast<EntityType>(-1);
using ComponentType = uint64_t;
using component_bitset = uint64_t;

} // namespace ecs
