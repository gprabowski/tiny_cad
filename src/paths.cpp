#include <paths.hpp>

#include <fstream>
#include <unordered_map>

#include <constructors.h>
#include <shader_manager.h>
#include <systems.h>

namespace paths {

template <int Resolution> struct int_pair_hash {
  std::size_t operator()(std::pair<int, int> const &v) const {
    return std::hash<int>()(v.first + v.second * Resolution);
  }
};

void generate_first_stage(float *data, int resolution, int size,
                          std::filesystem::path path) {
  float pixels_per_milimeter = resolution / 150.f;
  float halfsize = size / 2.f;
  int num_steps = 10;
  float step = halfsize / num_steps;
  float curr_step = step;
  int instruction = 1;

  std::ofstream output;
  output.open(path);

  auto sample = [&](int radius, float fx, float fy) {
    static std::unordered_map<std::pair<int, int>, float, int_pair_hash<8000>>
        cached;
    const int x = ((fx / size) + 0.5f) * resolution;
    const int y = ((fy / size) + 0.5f) * resolution;

    if (cached.count({x, y}) > 0) {
      return cached.at({x, y});
    }

    float curr_height = 0.f;

    int radius_in_pixels = radius * pixels_per_milimeter;
    for (int i = -radius_in_pixels; i < radius_in_pixels; ++i) {
      for (int j = -radius_in_pixels; i < radius_in_pixels; ++i) {
        const int final_x = x + i;
        const int final_y = y + j;
        if (final_x >= 0 && final_x < resolution && final_y >= 0 &&
            final_y < resolution) {
          curr_height =
              std::max(curr_height, data[final_y * resolution + final_x]);
        }
      }
    }

    cached[{x, y}] = 16.f + curr_height;
    return cached[{x, y}];
  };

  float x{-halfsize - 50}, y{-halfsize - 50}, z{50};

  output.setf(std::ios::fixed, std::ios::floatfield);
  output.precision(3);

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << z << std::endl;

  while (y <= halfsize) {
    x += curr_step;
    if ((x > halfsize && x - halfsize > step / 2) ||
        (x < -halfsize && (halfsize - x) > step / 2)) {
      x = x < 0 ? -halfsize : halfsize;
      output << "N" << instruction++ << "G01"
             << "X" << x << "Y" << y << "Z" << std::max(33.f, sample(8, x, y))
             << std::endl;
      y += step;
      output << "N" << instruction++ << "G01"
             << "X" << x << "Y" << y << "Z" << std::max(33.f, sample(8, x, y))
             << std::endl;
      curr_step = curr_step * -1;
      continue;
    }
    output << "N" << instruction++ << "G01"
           << "X" << x << "Y" << y << "Z" << std::max(33.f, sample(8, x, y))
           << std::endl;
  }

  while (y >= -halfsize) {
    x += curr_step;
    if ((x > halfsize && x - halfsize > step / 2) ||
        (x < -halfsize && (halfsize - x) > step / 2)) {
      x = x < 0 ? -halfsize : halfsize;
      output << "N" << instruction++ << "G01"
             << "X" << x << "Y" << y << "Z" << sample(8, x, y) << std::endl;
      y -= step;
      output << "N" << instruction++ << "G01"
             << "X" << x << "Y" << y << "Z" << sample(8, x, y) << std::endl;
      curr_step = curr_step * -1;
      continue;
    }
    output << "N" << instruction++ << "G01"
           << "X" << x << "Y" << y << "Z" << sample(8, x, y) << std::endl;
  }

  output.close();
}

ecs::EntityType find_named_c2_surface(const std::string &name) {
  auto &reg = ecs::registry::get_registry();

  for (auto &[idx, params] : reg.get_map<bspline_surface_params>()) {
    (void)params;
    auto &tmp_name = reg.get_component<tag_figure>(idx).name;
    if (tmp_name == name) {
      return idx;
    }
  }

  return -1;
}

void generate_second_stage(std::filesystem::path path) {
  static auto &reg = ecs::registry::get_registry();
  static auto &sm = shader_manager::get_manager();
  // 1. generate all intersections
  static systems::intersection_params params;

  const ecs::EntityType main_body = find_named_c2_surface("main_body");
  // const ecs::EntityType top = find_named_c2_surface("top");
  // const ecs::EntityType handle = find_named_c2_surface("handle");
  // const ecs::EntityType front = find_named_c2_surface("front");
  const ecs::EntityType plane = find_named_c2_surface("plane");

  auto &ctr = reg.get_component<transformation>(
      reg.get_map<cursor_params>().begin()->first);
  glm::vec3 cursor_pos = ctr.translation;

  sampler tmp_main_body_sampler = get_sampler(main_body);
  sampler main_body_sampler = tmp_main_body_sampler;
  sampler plane_sampler = get_sampler(plane);

  main_body_sampler.normal_translation = 5.f;

  main_body_sampler.sample = [&](float u, float v) -> glm::vec3 {
    return tmp_main_body_sampler.sample(u, v) +
           5.f * tmp_main_body_sampler.normal(u, v);
  };

  params.delta = 0.2f;
  params.subdivisions = 12;
  auto res = systems::intersect(main_body, plane, main_body_sampler,
                                plane_sampler, params, false, cursor_pos);
  const auto main_body_plane = res.idx;
  auto &g = reg.get_component<gl_object>(main_body_plane);
  constructors::add_icurve(
      g.points, sm.programs[shader_t::INTERPOLATION_CURVE_SHADER].idx);
}

} // namespace paths
