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
  float step = 8;

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

  x = -halfsize;
  y = -halfsize;
  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << std::max(33.f, sample(8, x, y))
         << std::endl;

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

  for (auto &[idx, tf] : reg.get_map<tag_figure>()) {
    auto &tmp_name = tf.name;
    if (tmp_name == name) {
      return idx;
    }
  }

  throw;
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

void generate_third_stage(std::filesystem::path path) {
  static auto &reg = ecs::registry::get_registry();
  //  R is 4.f

  // 1. generate intersection between main_body and the plane
  auto main_body_idx = find_named_c2_surface("main_body");
  auto front_idx = find_named_c2_surface("front");
  auto plane_idx = find_named_c2_surface("plane");

  sampler tmp_main_body_sampler = get_sampler(main_body_idx);
  sampler main_body_sampler = tmp_main_body_sampler;
  main_body_sampler.sample = [&](float u, float v) {
    return tmp_main_body_sampler.sample(u, v) +
           4.f * tmp_main_body_sampler.normal(u, v);
  };

  sampler tmp_plane_sampler = get_sampler(plane_idx);
  sampler plane_sampler = tmp_plane_sampler;
  plane_sampler.sample = [&](float u, float v) {
    return tmp_plane_sampler.sample(u, v) +
           4.f * tmp_plane_sampler.normal(u, v);
  };

  sampler tmp_front_sampler = get_sampler(front_idx);
  sampler front_sampler = tmp_front_sampler;
  front_sampler.sample = [&](float u, float v) {
    return tmp_front_sampler.sample(u, v) +
           4.f * tmp_front_sampler.normal(u, v);
  };

  auto main_body_plane =
      systems::intersect(main_body_idx, plane_idx, main_body_sampler,
                         plane_sampler, {}, false, {});

  auto main_body_front =
      systems::intersect(main_body_idx, front_idx, main_body_sampler,
                         front_sampler, {}, false, {});

  const auto main_body_intersect = main_body_plane.idx;
  auto &main_body_coords =
      reg.get_component<relationship>(main_body_intersect).virtual_children;

  const auto main_body_front_intersect = main_body_front.idx;
  auto &main_body_front_coords =
      reg.get_component<relationship>(main_body_front_intersect)
          .virtual_children;

  // create a table of min/max

  std::vector<std::vector<std::pair<float, float>>> main_body_holes(100);
  std::array<std::pair<float, float>, 100> front_tmp;
  main_body_table.fill({100.f, -100.f});

  std::array<std::pair<float, float>, 100> main_body_table;
  main_body_table.fill({100.f, -100.f});

  for (auto &c : main_body_front_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.f, 1.f);
    if (front_tmp[100 * t.x].size() > t.y) {
      front_tmp[100 * t.x].first = std::clamp(t.y, 0.f, 1.f);
    }

    if (front_tmp[100 * t.x].second < t.y) {
      front_tmp[100 * t.x].second = std::clamp(t.y, 0.f, 1.f);
    }
  }

  for (int ftmp = 0; ftmp < 100; ++ftmp) {
    if (front_tmp[ftmp].first < 1.f) {
      main_body_holes[ftmp].push_back(front_tmp[ftmp]);
    }
  }

  for (auto &v : main_body_holes) {
    std::sort(begin(v), end(v),
              [](const auto &a, const auto &b) { return a.first < b.first; })
  }

  for (auto &c : main_body_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.f, 1.f);
    if (main_body_table[100 * t.x].first > t.y) {
      main_body_table[100 * t.x].first = std::clamp(t.y, 0.f, 1.f);
    }

    if (main_body_table[100 * t.x].second < t.y) {
      main_body_table[100 * t.x].second = std::clamp(t.y, 0.f, 1.f);
    }
  }

  // prelude
  std::ofstream output;
  output.open(path);
  int instruction = 1;
  auto halfsize = 75.f;
  float x{-halfsize - 50}, y{-halfsize - 50}, z{50};

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << z << std::endl;

  x = 0.f;
  y = 0.f;

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << z << std::endl;

  int i = 0;

  auto iters = 0;
  while (i < 100) {
    ++iters;
    if (iters > 100) {
      throw;
    }

    while ((i < 100) && (main_body_table[i].first > 1.f ||
                         main_body_table[i].second < 0.f)) {
      ++i;
    }

    auto mix = [](float a, float b, float t) { return (1.f - t) * a + t * b; };

    bool started_even = (i % 2) == 0;
    if (main_body_table[i].first >= 0.f && main_body_table[i].first <= 1.f &&
        main_body_table[i].second >= 0.f && main_body_table[i].second <= 1.f) {
      auto s = main_body_sampler.sample(i / 99.f, main_body_table[i].first);

      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << std::endl;
    }

    while (i < 100 && (main_body_table[i].first >= 0.f &&
                       main_body_table[i].first <= 1.f &&
                       main_body_table[i].second >= 0.f &&
                       main_body_table[i].second <= 1.f)) {
      float tx = i / 99.f;
      // 1. find start position (not in hole)
      float start = std::min(main_body_table[i].first, 0.999f);
      // 2. if no holes on this x coord, go till the end
      float end = std::max(main_body_table[i].second, 0.001f);
      // 3. if holes here, iterate on holes

      auto is_hole = [&](float v) {
        for (auto &h : main_body_holes[i]) {
          if (v >= h.first && v <= h.second) {
            return true;
          }
        }
        return false;
      };

      auto round_hole = [&](float v) {
        for (auto &h : main_body_holes[i]) {
          if (v >= h.first && v <= h.second) {
            return h.second;
          }
        }
        return 1.f;
      };

      auto _j = 0.f;
      while (_j <= 1.f) {
        if (is_hole(_j)) {
          auto next_j = round_hole(_j);
          _j -= 0.01;
          auto mix_val = std::clamp(_j, 0.001f, 0.999f);
          auto s = main_body_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                                    ? mix(start, end, mix_val)
                                                    : mix(end, start, mix_val));
        }
        auto mix_val = std::clamp(j, 0.001f, 0.999f);
        auto s = main_body_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                                  ? mix(start, end, mix_val)
                                                  : mix(end, start, mix_val));

        if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
          throw;
        output << "N" << instruction++ << "G01"
               << "X" << s.x << "Y" << -s.z << "Z" << 16.f + s.y - 4.f
               << std::endl;
        _j += 0.01f;
      }
      ++i;
    }

    output << "N" << instruction++ << "G01"
           << "Z" << 50.f << std::endl;

    output << "N" << instruction++ << "G01"
           << "X" << 0.f << "Y" << 0.f << std::endl;
  }

  output.close();
}

} // namespace paths
