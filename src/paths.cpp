#include <paths.hpp>

#include <array>
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
  int num_steps = 50;
  float step = halfsize / num_steps;
  float curr_step = step;
  int instruction = 1;

  std::ofstream output;
  output.open(path);

  auto sample = [&](int radius, float fx, float fy) {
    static std::unordered_map<std::pair<int, int>, float, int_pair_hash<1500>>
        cached;
    const int x = ((fx / size) + 0.5f) * resolution;
    const int y = ((fy / size) + 0.5f) * resolution;

    if (cached.count({x, y}) > 0) {
      return cached.at({x, y});
    }

    float curr_height = 0.f;

    int radius_in_pixels = radius * pixels_per_milimeter;
    for (int i = -radius_in_pixels; i < radius_in_pixels; ++i) {
      for (int j = -radius_in_pixels; j < radius_in_pixels; ++j) {
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

ecs::EntityType find_named_point(const std::string &name) {
  auto &reg = ecs::registry::get_registry();

  for (auto &[idx, tf] : reg.get_map<tag_figure>()) {
    auto &tmp_name = tf.name;
    if (tmp_name == name) {
      return idx;
    }
  }

  throw;
  return -1;
}

void generate_second_stage(std::filesystem::path path) {
  static auto &reg = ecs::registry::get_registry();
  // 1. assume all intersections already generated
  // 2. build vector of positions
  std::vector<unsigned long> pi{
      find_named_point("first"),       find_named_point("second"),
      find_named_point("third"),       find_named_point("fourth"),
      find_named_point("fifth"),       find_named_point("sixth"),
      find_named_point("seventh"),     find_named_point("eigth"),
      find_named_point("ninth"),       find_named_point("tenth"),
      find_named_point("eleventh"),    find_named_point("twelth"),
      find_named_point("thirteenth"),  find_named_point("fourteenth"),
      find_named_point("fifteenth"),   find_named_point("sixteenth"),
      find_named_point("seventeenth"), find_named_point("eighteenth"),
      find_named_point("nineteenth"),  find_named_point("twentieth"),
      find_named_point("twentyfirst"), find_named_point("twentysecond"),
      find_named_point("twentythird"), find_named_point("twentyfourth"),
      find_named_point("twentyfifth"), find_named_point("twentysixth"),
  };

  std::vector<glm::vec3> points;
  for (unsigned long i = pi[0]; i <= pi[1]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  for (unsigned long i = pi[2]; i <= pi[3]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  points.push_back(reg.get_component<transformation>(pi[4]).translation);
  points.push_back(reg.get_component<transformation>(pi[5]).translation);

  for (unsigned long i = pi[6]; i <= pi[7]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  for (unsigned long i = pi[8]; i <= pi[9]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  for (unsigned long i = pi[10]; i <= pi[11]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  points.push_back(reg.get_component<transformation>(pi[12]).translation);

  for (unsigned long i = pi[13]; i <= pi[14]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  points.push_back(reg.get_component<transformation>(pi[15]).translation);

  for (unsigned long i = pi[16]; i <= pi[17]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  points.push_back(reg.get_component<transformation>(pi[18]).translation);

  for (unsigned long i = pi[19]; i <= pi[20]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  for (unsigned long i = pi[21]; i <= pi[22]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  for (unsigned long i = pi[23]; i <= pi[24]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  for (unsigned long i = pi[25]; i <= pi[0]; ++i) {
    points.push_back(reg.get_component<transformation>(i).translation);
  }

  // prelude
  std::ofstream output;
  output.open(path);
  int instruction = 1;
  auto halfsize = 75.f;
  float x{-halfsize - 50}, y{-halfsize - 50}, z{50};

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << z << std::endl;

  x = points[0].x;

  // generate
  z = 16;

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << z << std::endl;

  for (auto &p : points) {
    x = p.x;
    z = 16;
    y = -p.z;

    output << "N" << instruction++ << "G01"
           << "X" << x << "Y" << y << "Z" << z << std::endl;
  }

  x = points[0].x;
  z = 16;

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << z << std::endl;

  x = -halfsize - 50;
  y = -halfsize - 50;
  z = 16;

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << z << std::endl;

  x = -halfsize;
  y = -halfsize;
  z = 16;

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << z << std::endl;

  // create a table with min_max contour y value for each milimeter
  struct mm_data {
    float min{75}, max{-75};
  };

  std::vector<mm_data> y_values(151);
  for (size_t pidx = 1; pidx < points.size(); ++pidx) {
    auto &curr_point = points[pidx - 1];
    auto &next_point = points[pidx];

    if (glm::length(curr_point - next_point) > 0.9f) {
      const int num_div = glm::length(curr_point - next_point) / 0.5f;

      for (int i = 0; i <= num_div; ++i) {
        const float frac = i / static_cast<float>(num_div);
        auto const inter_point = (1.f - frac) * curr_point + frac * next_point;

        auto cmin = y_values[static_cast<int>(inter_point.x) + 75].min;
        auto cmax = y_values[static_cast<int>(inter_point.x) + 75].max;

        y_values[static_cast<int>(inter_point.x) + 75].min =
            std::min(cmin, -inter_point.z);
        y_values[static_cast<int>(inter_point.x) + 75].max =
            std::max(cmax, -inter_point.z);
      }
    } else {
      auto cmin = y_values[static_cast<int>(curr_point.x) + 75].min;
      auto cmax = y_values[static_cast<int>(curr_point.x) + 75].max;

      y_values[static_cast<int>(curr_point.x) + 75].min =
          std::min(cmin, -curr_point.z);
      y_values[static_cast<int>(curr_point.x) + 75].max =
          std::max(cmax, -curr_point.z);
    }
  }

  // jump every 5 mm
  for (int px = -75; px <= 75; px += 5) {
    // 1. jump to min y
    x = px;
    y = y_values[px + 75].min;
    z = 16;

    output << "N" << instruction++ << "G01"
           << "X" << x << "Y" << y << "Z" << z << std::endl;

    y = -75;

    output << "N" << instruction++ << "G01"
           << "X" << x << "Y" << y << "Z" << z << std::endl;

    output << "N" << instruction++ << "G01"
           << "X" << x + 5.f << "Y" << y << "Z" << z << std::endl;
  }

  x = 75;

  output << "N" << instruction++ << "G01"
         << "X" << 75 << "Y" << y << "Z" << z << std::endl;

  y = 75;

  output << "N" << instruction++ << "G01"
         << "X" << 75 << "Y" << y << "Z" << z << std::endl;

  // jump every 5 mm
  for (int px = 75; px >= -75; px -= 5) {
    // 1. jump to min y
    x = px;
    y = y_values[px + 75].max;
    z = 16;

    output << "N" << instruction++ << "G01"
           << "X" << x << "Y" << y << "Z" << z << std::endl;

    y = 75;

    output << "N" << instruction++ << "G01"
           << "X" << x << "Y" << y << "Z" << z << std::endl;

    output << "N" << instruction++ << "G01"
           << "X" << x - 5.f << "Y" << y << "Z" << z << std::endl;
  }

  x = -100;

  output << "N" << instruction++ << "G01"
         << "X" << x - 5.f << "Y" << y << "Z" << z << std::endl;

  x = -halfsize - 50;
  y = -halfsize - 50;
  z = 16;

  output.close();
}

} // namespace paths
