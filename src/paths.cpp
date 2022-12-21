#include <paths.hpp>

#include <array>
#include <fstream>
#include <unordered_map>

#include <constructors.h>
#include <shader_manager.h>
#include <systems.h>

#include <log.h>

namespace paths {

void out(std::ofstream &output, int instruction, float x, float y, float z) {
  static float last_x{-100.f}, last_y{-100.f}, last_z{-100.f};

  z = std::max(z, 16.f);

  bool xsame = (std::abs(x - last_x) < 1e-3);
  bool ysame = (std::abs(y - last_y) < 1e-3);
  bool zsame = (std::abs(z - last_z) < 1e-3);

  // all are the same
  if (xsame && ysame && zsame) {
    return;
  }

  // two are the same
  if (!xsame && ysame && zsame) {
    last_x = x;
    output << "N" << instruction << "G01"
           << "X" << x << std::endl;
    return;
  }

  if (xsame && !ysame && zsame) {
    last_y = y;
    output << "N" << instruction << "G01"
           << "Y" << y << std::endl;
    return;
  }

  if (xsame && ysame && !zsame) {
    last_z = z;
    output << "N" << instruction << "G01"
           << "Z" << z << std::endl;
    return;
  }

  // one is the same
  if (xsame && !ysame && !zsame) {
    last_y = y;
    last_z = z;
    output << "N" << instruction << "G01"
           << "Y" << y << "Z" << z << std::endl;
    return;
  }

  if (!xsame && ysame && !zsame) {
    last_x = x;
    last_z = z;
    output << "N" << instruction << "G01"
           << "X" << x << "Z" << z << std::endl;
    return;
  }

  if (!xsame && !ysame && zsame) {
    last_x = x;
    last_y = y;
    output << "N" << instruction << "G01"
           << "X" << x << "Y" << y << std::endl;
    return;
  }

  if (!xsame && !ysame && !zsame) {
    last_x = x;
    last_y = y;
    last_z = z;
    output << "N" << instruction << "G01"
           << "X" << x << "Y" << y << "Z" << z << std::endl;
    return;
  }
}

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
  output.setf(std::ios::fixed, std::ios::floatfield);
  output.precision(3);
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
    return cached[{x, y}] + 0.5f;
  };

  float x{-halfsize - 50}, y{-halfsize - 50}, z{66.f};

  output.setf(std::ios::fixed, std::ios::floatfield);
  output.precision(3);

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << "Z" << 66.f << std::endl;

  out(output, instruction++, x, y, z);

  x = -halfsize;
  y = -halfsize;
  out(output, instruction++, x, y, std::max(35.f, sample(8, x, y)));

  while (y <= halfsize) {
    x += curr_step;
    if ((x > halfsize && x - halfsize > step / 2) ||
        (x < -halfsize && (halfsize - x) > step / 2)) {
      x = x < 0 ? -halfsize : halfsize;
      out(output, instruction++, x, y, std::max(35.f, sample(8, x, y)));
      y += step;
      out(output, instruction++, x, y, std::max(35.f, sample(8, x, y)));
      curr_step = curr_step * -1;
      continue;
    }
    out(output, instruction++, x, y, std::max(35.f, sample(8, x, y)));
  }

  while (y >= -halfsize) {
    x += curr_step;
    if ((x > halfsize && x - halfsize > step / 2) ||
        (x < -halfsize && (halfsize - x) > step / 2)) {
      x = x < 0 ? -halfsize : halfsize;
      out(output, instruction++, x, y, 0.3f + sample(8, x, y));
      y -= step;
      out(output, instruction++, x, y, 0.3f + sample(8, x, y));
      curr_step = curr_step * -1;
      continue;
    }
    out(output, instruction++, x, y, 0.3f + sample(8, x, y));
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;

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
  output.setf(std::ios::fixed, std::ios::floatfield);
  output.precision(3);
  output.open(path);
  int instruction = 1;
  auto halfsize = 75.f;
  float x{-halfsize - 50}, y{-halfsize - 50}, z{66};

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << "Z" << 66.f << std::endl;

  out(output, instruction++, x, y, z);

  x = points[0].x;

  // generate
  z = 16;

  out(output, instruction++, x, y, z);

  for (auto &p : points) {
    x = p.x;
    z = 16;
    y = -p.z;

    out(output, instruction++, x, y, z);
  }

  x = points[0].x;
  z = 16;

  out(output, instruction++, x, y, z);

  x = -halfsize - 50;
  y = -halfsize - 50;
  z = 16;

  out(output, instruction++, x, y, z);

  x = -halfsize;
  y = -halfsize;
  z = 16;

  out(output, instruction++, x, y, z);

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
    y = y_values[px + 75].min - 2;
    z = 16;

    out(output, instruction++, x, y, z);

    if (px + 80 < static_cast<int>(y_values.size())) {

      auto nv = y_values[px + 80].min - 2;
      auto av = (y + nv) / 2.f;

      bool must_skip =
          (av >= y_values[px + 76].min && av <= y_values[px + 76].max) ||
          (av >= y_values[px + 77].min && av <= y_values[px + 77].max) ||
          (av >= y_values[px + 78].min && av <= y_values[px + 78].max) ||
          (av >= y_values[px + 79].min && av <= y_values[px + 79].max);

      if (!must_skip) {
        px += 5;
        x = px;
        y = y_values[px + 75].min - 2;
        out(output, instruction++, x, y, z);
      }
    }

    y = -75;

    out(output, instruction++, x, y, z);
    out(output, instruction++, x + 5.f, y, z);
  }

  x = 75;

  out(output, instruction++, 75, y, z);

  y = 75;

  out(output, instruction++, 75, y, z);

  // jump every 5 mm
  for (int px = 75; px >= -75; px -= 5) {
    // 1. jump to min y
    x = px;
    y = y_values[px + 75].max + 2;
    z = 16;

    out(output, instruction++, x, y, z);
    if (px != -65 && px + 70 >= 0) {

      auto nv = y_values[px + 70].max + 2;
      auto av = (y + nv) / 2.f;

      bool must_skip =
          (av >= y_values[px + 74].min && av <= y_values[px + 74].max) ||
          (av >= y_values[px + 73].min && av <= y_values[px + 73].max) ||
          (av >= y_values[px + 72].min && av <= y_values[px + 72].max) ||
          (av >= y_values[px + 71].min && av <= y_values[px + 71].max);

      if (!must_skip) {
        px -= 5;
        x = px;
        y = y_values[px + 75].max + 2;
        out(output, instruction++, x, y, z);
      }
    }

    y = 75;

    out(output, instruction++, x, y, z);
    out(output, instruction++, x - 5.f, y, z);
  }

  x = -100;

  out(output, instruction++, x - 5.f, y, z);

  x = -halfsize - 50;
  y = -halfsize - 50;
  z = 16;

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;

  output.close();
}

void generate_main_body_detail(std::ofstream &output, int &instruction) {
  auto &reg = ecs::registry::get_registry();
  // 1. generate intersection between main_body and the plane
  auto main_body_idx = find_named_c2_surface("main_body");
  auto front_idx = find_named_c2_surface("front");
  auto top_idx = find_named_c2_surface("top");
  auto handle_idx = find_named_c2_surface("handle");
  auto plane_idx = find_named_c2_surface("plane");

  sampler tmp_main_body_sampler = get_sampler(main_body_idx);
  sampler main_body_sampler = tmp_main_body_sampler;
  main_body_sampler.sample = [&](float u, float v) {
    return tmp_main_body_sampler.sample(u, v) +
           4.f * tmp_main_body_sampler.normal(u, v);
  };

  sampler tmp_top_sampler = get_sampler(top_idx);
  sampler top_sampler = tmp_top_sampler;
  top_sampler.sample = [&](float u, float v) {
    return tmp_top_sampler.sample(u, v) + 4.f * tmp_top_sampler.normal(u, v);
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

  sampler tmp_handle_sampler = get_sampler(handle_idx);
  sampler handle_sampler = tmp_handle_sampler;
  handle_sampler.sample = [&](float u, float v) {
    return tmp_handle_sampler.sample(u, v) +
           4.f * tmp_handle_sampler.normal(u, v);
  };

  auto main_body_plane =
      systems::intersect(main_body_idx, plane_idx, main_body_sampler,
                         plane_sampler, {}, false, {});

  auto main_body_front =
      systems::intersect(main_body_idx, front_idx, main_body_sampler,
                         front_sampler, {}, false, {});

  auto main_body_top = systems::intersect(
      main_body_idx, top_idx, main_body_sampler, top_sampler, {}, false, {});

  systems::intersection_params params{};
  params.subdivisions = 12;

  auto &cursor_t = reg.get_component<transformation>(
                          reg.get_map<cursor_params>().begin()->first)
                       .translation;
  cursor_t = {44.403, -0.767, 25.561};

  auto main_body_handle_top =
      systems::intersect(main_body_idx, handle_idx, main_body_sampler,
                         handle_sampler, params, false, cursor_t);

  params.start_from_cursor = true;
  auto main_body_handle_bottom =
      systems::intersect(main_body_idx, handle_idx, main_body_sampler,
                         handle_sampler, params, false, cursor_t);

  const auto main_body_intersect = main_body_plane.idx;
  auto &main_body_coords =
      reg.get_component<relationship>(main_body_intersect).virtual_children;

  const auto main_body_front_intersect = main_body_front.idx;
  auto &main_body_front_coords =
      reg.get_component<relationship>(main_body_front_intersect)
          .virtual_children;

  const auto main_body_top_intersect = main_body_top.idx;
  auto &main_body_top_coords =
      reg.get_component<relationship>(main_body_top_intersect).virtual_children;

  const auto main_body_handle_top_intersect = main_body_handle_top.idx;
  auto &main_body_handle_top_coords =
      reg.get_component<relationship>(main_body_handle_top_intersect)
          .virtual_children;

  const auto main_body_handle_bottom_intersect = main_body_handle_bottom.idx;
  auto &main_body_handle_bottom_coords =
      reg.get_component<relationship>(main_body_handle_bottom_intersect)
          .virtual_children;

  // create a table of min/max
  std::array<std::pair<float, float>, 200> main_body_table;
  main_body_table.fill({100.f, -100.f});

  std::array<std::pair<float, float>, 200> front_hole;
  front_hole.fill({100.f, -100.f});

  std::array<std::pair<float, float>, 200> handle_bottom_hole;
  handle_bottom_hole.fill({100.f, -100.f});

  std::array<std::pair<float, float>, 200> handle_top_hole;
  handle_top_hole.fill({100.f, -100.f});

  for (std::size_t hi = 0; hi < main_body_coords.size(); ++hi) {
    auto &pc = main_body_coords[hi];
    auto &nc = main_body_coords[hi];

    auto &pt = reg.get_component<transformation>(pc).translation;
    auto &nt = reg.get_component<transformation>(nc).translation;

    for (int h = 0; h < 5; ++h) {
      auto t = glm::mix(pt, nt, h / 4.f);
      t.x = std::clamp(t.x, 0.f, 1.f);
      if (main_body_table[200 * t.x].first > t.y) {
        main_body_table[200 * t.x].first = std::clamp(t.y, 0.f, 1.f);
      }

      if (main_body_table[200 * t.x].second < t.y) {
        main_body_table[200 * t.x].second = std::clamp(t.y, 0.f, 1.f);
      }
    }
  }

  for (std::size_t hi = 0; hi < main_body_top_coords.size(); ++hi) {
    auto &pc = main_body_top_coords[hi];
    auto &nc = main_body_top_coords[hi];

    auto &pt = reg.get_component<transformation>(pc).translation;
    auto &nt = reg.get_component<transformation>(nc).translation;

    for (int h = 0; h < 5; ++h) {
      auto t = glm::mix(pt, nt, h / 4.f);

      t.x = std::clamp(t.x, 0.f, 1.f);

      if (main_body_table[200 * t.x].first < 1.f &&
          main_body_table[200 * t.x].second > 0.f) {
        if (main_body_table[200 * t.x].second > t.y) {
          main_body_table[200 * t.x].second = std::clamp(t.y, 0.f, 1.f);
        }
      }
    }
  }

  for (std::size_t hi = 0; hi < main_body_front_coords.size(); ++hi) {
    auto &pc = main_body_front_coords[hi];
    auto &nc = main_body_front_coords[hi];

    auto &pt = reg.get_component<transformation>(pc).translation;
    auto &nt = reg.get_component<transformation>(nc).translation;

    for (int h = 0; h < 5; ++h) {
      auto t = glm::mix(pt, nt, h / 4.f);

      t.x = std::clamp(t.x, 0.f, 1.f);
      if (front_hole[200 * t.x].first > t.y) {
        front_hole[200 * t.x].first = std::clamp(t.y, 0.f, 1.f);
      }

      if (front_hole[200 * t.x].second < t.y) {
        front_hole[200 * t.x].second = std::clamp(t.y, 0.f, 1.f);
      }
    }
  }

  for (std::size_t hi = 1; hi < main_body_handle_top_coords.size(); ++hi) {
    auto &pc = main_body_handle_top_coords[hi];
    auto &nc = main_body_handle_top_coords[hi];

    auto &pt = reg.get_component<transformation>(pc).translation;
    auto &nt = reg.get_component<transformation>(nc).translation;

    for (int h = 0; h < 5; ++h) {
      auto t = glm::mix(pt, nt, h / 4.f);
      t.x = std::clamp(t.x, 0.f, 1.f);
      if (handle_top_hole[std::clamp(200 * t.x, 0.f, 199.f)].first > t.y) {
        handle_top_hole[std::clamp(200 * t.x, 0.f, 199.f)].first =
            std::clamp(t.y, 0.f, 1.f);
      }

      if (handle_top_hole[std::clamp(200 * t.x, 0.f, 199.f)].second < t.y) {
        handle_top_hole[std::clamp(200 * t.x, 0.f, 199.f)].second =
            std::clamp(t.y, 0.f, 1.f);
      }
    }
  }

  for (std::size_t hi = 1; hi < main_body_handle_bottom_coords.size(); ++hi) {
    auto &pc = main_body_handle_bottom_coords[hi];
    auto &nc = main_body_handle_bottom_coords[hi];

    auto &pt = reg.get_component<transformation>(pc).translation;
    auto &nt = reg.get_component<transformation>(nc).translation;

    for (int h = 0; h < 5; ++h) {
      auto t = glm::mix(pt, nt, h / 4.f);

      t.x = std::clamp(t.x, 0.f, 1.f);
      if (handle_bottom_hole[std::clamp(200 * t.x, 0.f, 199.f)].first > t.y) {
        handle_bottom_hole[std::clamp(200 * t.x, 0.f, 199.f)].first =
            std::clamp(t.y, 0.f, 1.f);
      }

      if (handle_bottom_hole[std::clamp(200 * t.x, 0.f, 199.f)].second < t.y) {
        handle_bottom_hole[std::clamp(200 * t.x, 0.f, 199.f)].second =
            std::clamp(t.y, 0.f, 1.f);
      }
    }
  }

  for (int timp = 0; timp < 200; ++timp) {
    if (main_body_table[timp].first > 2.f ||
        main_body_table[timp].second < -1.f) {
      continue;
    }

    if (front_hole[timp].first < 1.f &&
        front_hole[timp].second >= main_body_table[timp].second) {
      main_body_table[timp].second = front_hole[timp].first;
      front_hole[timp] = {100.f, -100.f};
    }

    if (front_hole[timp].second > 0.f &&
        front_hole[timp].first <= main_body_table[timp].first) {
      main_body_table[timp].first = front_hole[timp].second;
      front_hole[timp] = {100.f, -100.f};
    }

    if (handle_top_hole[timp].first <= 1.f &&
        handle_top_hole[timp].second >= main_body_table[timp].second) {

      if (timp != 166 && timp != 167 && timp != 170) {
        main_body_table[timp].second = handle_top_hole[timp].first;
        handle_top_hole[timp] = {100.f, -100.f};
      }
    }

    if (handle_bottom_hole[timp].second >= 0.f &&
        handle_bottom_hole[timp].first <= main_body_table[timp].first) {
      main_body_table[timp].first = handle_bottom_hole[timp].second;
      handle_bottom_hole[timp] = {100.f, -100.f};
    }
  }

  int i = 0;

  auto mix = [](float a, float b, float t) { return (1.f - t) * a + t * b; };

  bool started_even = (i % 2) == 0;
  if (main_body_table[i].first >= 0.f && main_body_table[i].first <= 1.f &&
      main_body_table[i].second >= 0.f && main_body_table[i].second <= 1.f) {
    auto s = main_body_sampler.sample(i / 199.f, main_body_table[i].first);

    out(output, instruction++, s.x, -s.z, 66.f);
  }

  while (i < 200 && (main_body_table[i + 1].first >= 0.f &&
                     main_body_table[i + 1].first <= 1.f &&
                     main_body_table[i + 1].second >= 0.f &&
                     main_body_table[i + 1].second <= 1.f)) {
    float tx = i / 199.f;
    // 1. find start position (not in hole)

    if (front_hole[i].first > 1.f || front_hole[i].second < 0.f) {
      float start = std::max(main_body_table[i].first, 0.001f);
      float end = std::min(main_body_table[i].second, 0.999f);
      for (int j = 0; j < 100; ++j) {
        auto mix_val = std::clamp(j / 99.f, 0.001f, 0.999f);
        auto s = main_body_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                                  ? mix(start, end, mix_val)
                                                  : mix(end, start, mix_val));

        if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
          throw;
        out(output, instruction++, s.x, -s.z, 12.f + s.y);
      }
    } else {
      float start = std::max(main_body_table[i].first, 0.001f);
      float middle_start = std::max(front_hole[i].first, 0.001f);

      float middle_end = std::min(front_hole[i].second, 0.999f);
      float end = std::min(main_body_table[i].second, 0.999f);

      // go till hole
      for (int j = 0; j < 40; ++j) {
        auto mix_val = std::clamp(j / 39.f, 0.001f, 0.999f);
        auto s =
            main_body_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                             ? mix(start, middle_start, mix_val)
                                             : mix(end, middle_end, mix_val));

        if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
          throw;
        out(output, instruction++, s.x, -s.z, 12.f + s.y);
      }
      // jump over hole
      output << "N" << instruction++ << "G01"
             << "Z" << 35.f << std::endl;

      auto s = main_body_sampler.sample(
          tx, (started_even == ((i % 2) == 0)) ? middle_end : middle_start);

      out(output, instruction++, s.x, -s.z, 35.f);

      out(output, instruction++, s.x, -s.z, 12.f + s.y);

      for (int j = 0; j < 40; ++j) {
        auto mix_val = std::clamp(j / 39.f, 0.001f, 0.999f);
        auto s = main_body_sampler.sample(
            tx, (started_even == ((i % 2) == 0))
                    ? mix(middle_end, end, mix_val)
                    : mix(middle_start, start, mix_val));

        if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
          throw;
        out(output, instruction++, s.x, -s.z, 12.f + s.y);
      }
    }
    ++i;
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;

  i = 199;
  started_even = (i % 2) == 0;
  if (main_body_table[i].first >= 0.f && main_body_table[i].first <= 1.f &&
      main_body_table[i].second >= 0.f && main_body_table[i].second <= 1.f) {
    auto s = main_body_sampler.sample(1.f, main_body_table[i].first);

    out(output, instruction++, s.x, -s.z, 66.f);
  }

  // between 171 and 170
  // go from the right, two holes here
  while (i < 200 &&
         (main_body_table[i].first >= 0.f && main_body_table[i].first <= 1.f &&
          main_body_table[i].second >= 0.f &&
          main_body_table[i].second <= 1.f)) {
    TINY_CAD_INFO("i: {0}", i);
    float tx = i / 199.f;
    if (handle_top_hole[i].first <= 1.f && handle_bottom_hole[i].first <= 1.f) {

      float start = std::max(main_body_table[i].first, 0.001f);

      float first_start = std::max(handle_bottom_hole[i].first, 0.001f);
      float first_end = std::min(handle_bottom_hole[i].second, 0.999f);

      float second_start = std::max(handle_top_hole[i].first, 0.001f);
      float second_end = std::min(handle_top_hole[i].second, 0.999f);

      float end = std::min(main_body_table[i].second, 0.999f);

      // go till hole
      for (int j = 0; j < 40; ++j) {
        auto mix_val = std::clamp(j / 39.f, 0.001f, 0.999f);
        auto s =
            main_body_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                             ? mix(start, first_start, mix_val)
                                             : mix(end, second_end, mix_val));

        if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
          throw;
        out(output, instruction++, s.x, -s.z, 12.f + s.y);
      }
      // jump over first hole
      output << "N" << instruction++ << "G01"
             << "Z" << 35.f << std::endl;
      auto s = main_body_sampler.sample(
          tx, (started_even == ((i % 2) == 0)) ? first_end : second_start);
      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << std::endl;

      out(output, instruction++, s.x, -s.z, 12.f + s.y);

      for (int j = 0; j < 40; ++j) {
        auto mix_val = std::clamp(j / 39.f, 0.001f, 0.999f);
        auto s = main_body_sampler.sample(
            tx, (started_even == ((i % 2) == 0))
                    ? mix(first_end, second_start, mix_val)
                    : mix(second_start, first_end, mix_val));

        if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
          throw;
        out(output, instruction++, s.x, -s.z, 12.f + s.y);
      }
      // jump over second hole
      output << "N" << instruction++ << "G01"
             << "Z" << 35.f << std::endl;
      s = main_body_sampler.sample(
          tx, (started_even == ((i % 2) == 0)) ? second_end : first_start);
      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << std::endl;

      out(output, instruction++, s.x, -s.z, 12.f + s.y);

      for (int j = 0; j < 40; ++j) {
        auto mix_val = std::clamp(j / 39.f, 0.001f, 0.999f);
        auto s = main_body_sampler.sample(
            tx, (started_even == ((i % 2) == 0))
                    ? mix(second_end, end, mix_val)
                    : mix(first_start, start, mix_val));

        if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
          throw;
        out(output, instruction++, s.x, -s.z, 12.f + s.y);
      }
    } else if (handle_top_hole[i].first <= 1.f &&
               handle_bottom_hole[i].first > 1.f) {
      float start = std::max(main_body_table[i].first, 0.001f);
      float middle_start = std::max(handle_top_hole[i].first, 0.001f);

      float middle_end = std::min(handle_top_hole[i].second, 0.999f);
      float end = std::min(main_body_table[i].second, 0.999f);

      if (i != 168 && i != 169 && i != 170) {
        // go till hole
        for (int j = 0; j < 40; ++j) {
          auto mix_val = std::clamp(j / 39.f, 0.001f, 0.999f);
          auto s = main_body_sampler.sample(
              tx, (started_even == ((i % 2) == 0))
                      ? mix(start, middle_start, mix_val)
                      : mix(end, middle_end, mix_val));

          if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
            throw;
          out(output, instruction++, s.x, -s.z, 12.f + s.y);
        }
      }
      // jump over hole
      output << "N" << instruction++ << "G01"
             << "Z" << 35.f << std::endl;
      auto s = main_body_sampler.sample(
          tx, (started_even == ((i % 2) == 0)) ? middle_end : middle_start);
      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << std::endl;
      out(output, instruction++, s.x, -s.z, 12.f + s.y);

      for (int j = 0; j < 40; ++j) {
        auto mix_val = std::clamp(j / 39.f, 0.001f, 0.999f);
        auto s = main_body_sampler.sample(
            tx, (started_even == ((i % 2) == 0))
                    ? mix(middle_end, end, mix_val)
                    : mix(middle_start, start, mix_val));

        if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
          throw;
        out(output, instruction++, s.x, -s.z, 12.f + s.y);
      }
    } else if (handle_top_hole[i].first > 2.f &&
               handle_bottom_hole[i].first > 2.f) {
      // first option - no intersection
      float start = std::min(main_body_table[i].first, 0.999f);
      float end = std::max(main_body_table[i].second, 0.001f);

      for (int j = 0; j < 100; ++j) {
        auto mix_val = std::clamp(j / 99.f, 0.001f, 0.999f);
        auto s = main_body_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                                  ? mix(start, end, mix_val)
                                                  : mix(end, start, mix_val));

        if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
          throw;
        out(output, instruction++, s.x, -s.z, 12.f + s.y);
      }
    }
    --i;
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;
}

void generate_front_detail(std::ofstream &output, int &instruction) {
  auto &reg = ecs::registry::get_registry();
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

  systems::intersection_params params{};
  auto &cursor_t = reg.get_component<transformation>(
                          reg.get_map<cursor_params>().begin()->first)
                       .translation;
  cursor_t = {-48.501, 3.828, 9.344};
  params.start_from_cursor = true;

  auto front_plane_bottom =
      systems::intersect(front_idx, plane_idx, front_sampler, plane_sampler,
                         params, false, cursor_t);

  params.subdivisions = 12;
  cursor_t = {-47.018, 4.066, -10.705};
  auto front_plane_top =
      systems::intersect(front_idx, plane_idx, front_sampler, plane_sampler,
                         params, false, cursor_t);

  auto front_main_body =
      systems::intersect(front_idx, main_body_idx, front_sampler,
                         main_body_sampler, {}, false, cursor_t);

  const auto front_main_body_intersect = front_main_body.idx;
  auto &front_main_body_coords =
      reg.get_component<relationship>(front_main_body_intersect)
          .virtual_children;

  const auto front_plane_top_intersect = front_plane_top.idx;
  auto &front_plane_top_coords =
      reg.get_component<relationship>(front_plane_top_intersect)
          .virtual_children;

  const auto front_plane_bottom_intersect = front_plane_bottom.idx;
  auto &front_plane_bottom_coords =
      reg.get_component<relationship>(front_plane_bottom_intersect)
          .virtual_children;

  // create a table of min/max
  std::array<std::pair<float, float>, 100> front_table;
  front_table.fill({0.001f, 0.999f});

  for (auto &c : front_main_body_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.001f, 0.999f);
    front_table[100 * t.x].first = std::clamp(t.y, 0.001f, 0.999f);
  }

  int topmin = 101, topmax = -1;
  for (auto &c : front_plane_top_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.001f, 0.999f);

    if (100 * t.x < topmin) {
      topmin = 100 * t.x;
    }

    if (100 * t.x > topmax) {
      topmax = 100 * t.x;
    }

    if (t.y <= front_table[100 * t.x].first) {
      front_table[100 * t.x] = {100.f, -100.f};
    } else {
      front_table[100 * t.x].second = std::clamp(t.y, 0.f, 1.f);
    }
  }

  int bottommin = 101, bottommax = -1;
  for (auto &c : front_plane_bottom_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.f, 1.f);

    if (100 * t.x < bottommin) {
      bottommin = 100 * t.x;
    }

    if (100 * t.x > bottommax) {
      bottommax = 100 * t.x;
    }

    if (t.y <= front_table[100 * t.x].first) {
      front_table[100 * t.x] = {100.f, -100.f};
    } else {
      front_table[100 * t.x].second = std::clamp(t.y, 0.f, 1.f);
    }
  }

  int i = 0;

  auto mix = [](float a, float b, float t) { return (1.f - t) * a + t * b; };

  bool started_even = (i % 2) == 0;

  if (front_table[i].first >= 0.f && front_table[i].first <= 1.f &&
      front_table[i].second >= 0.f && front_table[i].second <= 1.f) {
    auto s = front_sampler.sample(i / 99.f, front_table[i].first);

    out(output, instruction++, s.x, -s.z, 66.f);
  }

  for (; i < bottommax; ++i) {
    if (front_table[i].first > 1.f) {
      break;
    }
    float tx = i / 99.f;
    // first option - no intersection
    float start = std::min(front_table[i].first, 0.999f);
    float end = std::max(front_table[i].second, 0.001f);

    for (int j = 0; j < 20; ++j) {
      auto mix_val = std::clamp(j / 19.f, 0.001f, 0.999f);
      auto s = front_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                            ? mix(start, end, mix_val)
                                            : mix(end, start, mix_val));

      if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
        throw;
      out(output, instruction++, s.x, -s.z, 12.f + s.y);
    }
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 35.f << std::endl;

  i = topmin;
  while (front_table[i].first > 1.f) {
    ++i;
  }

  if (front_table[i].first >= 0.f && front_table[i].first <= 1.f &&
      front_table[i].second >= 0.f && front_table[i].second <= 1.f) {
    auto s = front_sampler.sample(i / 99.f, front_table[i].first);

    out(output, instruction++, s.x, -s.z, 35.f);
  }

  for (; i < 100; ++i) {
    if (front_table[i].first > 1.f) {
      continue;
    }
    float tx = i / 99.f;
    // first option - no intersection
    float start = std::min(front_table[i].first, 0.999f);
    float end = std::max(front_table[i].second, 0.001f);

    for (int j = 0; j < 20; ++j) {
      auto mix_val = std::clamp(j / 19.f, 0.001f, 0.999f);
      auto s = front_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                            ? mix(start, end, mix_val)
                                            : mix(end, start, mix_val));

      if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
        throw;
      out(output, instruction++, s.x, -s.z, 12.f + s.y);
    }
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;
}

void generate_handle_detail(std::ofstream &output, int &instruction) {
  auto &reg = ecs::registry::get_registry();
  // 1. generate intersection between main_body and the plane
  auto main_body_idx = find_named_c2_surface("main_body");
  auto handle_idx = find_named_c2_surface("handle");
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

  sampler tmp_handle_sampler = get_sampler(handle_idx);
  sampler handle_sampler = tmp_handle_sampler;
  handle_sampler.sample = [&](float u, float v) {
    return tmp_handle_sampler.sample(u, v) +
           4.f * tmp_handle_sampler.normal(u, v);
  };

  systems::intersection_params params{};
  auto &cursor_t = reg.get_component<transformation>(
                          reg.get_map<cursor_params>().begin()->first)
                       .translation;

  auto handle_plane_outer =
      systems::intersect(handle_idx, plane_idx, handle_sampler, plane_sampler,
                         {}, false, cursor_t);

  params.start_from_cursor = true;
  cursor_t = {36.917, 4.108, 10.137};
  auto handle_plane_inner =
      systems::intersect(handle_idx, plane_idx, handle_sampler, plane_sampler,
                         params, false, cursor_t);

  params.start_from_cursor = false;
  params.subdivisions = 12;
  auto handle_main_body_top =
      systems::intersect(handle_idx, main_body_idx, handle_sampler,
                         main_body_sampler, params, false, cursor_t);

  params.start_from_cursor = true;
  cursor_t = {43.042, 5.005, 23.815};

  auto handle_main_body_bottom =
      systems::intersect(handle_idx, main_body_idx, handle_sampler,
                         main_body_sampler, params, false, cursor_t);

  const auto handle_main_body_top_intersect = handle_main_body_top.idx;
  auto &handle_main_body_top_coords =
      reg.get_component<relationship>(handle_main_body_top_intersect)
          .virtual_children;

  const auto handle_main_body_bottom_intersect = handle_main_body_bottom.idx;
  auto &handle_main_body_bottom_coords =
      reg.get_component<relationship>(handle_main_body_bottom_intersect)
          .virtual_children;

  const auto handle_plane_outer_intersect = handle_plane_outer.idx;
  auto &handle_plane_outer_coords =
      reg.get_component<relationship>(handle_plane_outer_intersect)
          .virtual_children;

  const auto handle_plane_inner_intersect = handle_plane_inner.idx;
  auto &handle_plane_inner_coords =
      reg.get_component<relationship>(handle_plane_inner_intersect)
          .virtual_children;

  // create a table of min/max
  std::array<std::pair<float, float>, 100> handle_table;
  handle_table.fill({0.001f, 0.999f});

  for (auto &c : handle_main_body_top_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.001f, 0.999f);
    handle_table[100 * t.x].first = std::clamp(t.y, 0.001f, 0.999f);
  }

  for (std::size_t ci = 1; ci < handle_main_body_bottom_coords.size(); ++ci) {
    auto &pt = reg.get_component<transformation>(
                      handle_main_body_bottom_coords[ci - 1])
                   .translation;
    auto &nt =
        reg.get_component<transformation>(handle_main_body_bottom_coords[ci])
            .translation;

    for (int i = 0; i < 5; ++i) {
      auto t = glm::mix(pt, nt, i / 4.f);
      t.x = std::clamp(t.x, 0.001f, 0.999f);
      handle_table[100 * t.x].second = std::clamp(t.y, 0.001f, 0.999f);
    }
  }

  int outermin = 101, outermax = -1;
  for (auto &c : handle_plane_outer_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.001f, 0.999f);

    if (100 * t.x < outermin) {
      outermin = 100 * t.x;
    }

    if (100 * t.x > outermax) {
      outermax = 100 * t.x;
    }

    if (t.y <= handle_table[100 * t.x].first) {
      handle_table[100 * t.x] = {100.f, -100.f};
    } else if (handle_table[100 * t.x].second > 0.f) {
      handle_table[100 * t.x].second =
          std::min(std::clamp(t.y, 0.f, 1.f), handle_table[100 * t.x].second);
    }
  }

  int innermin = 101, innermax = -1;
  std::array<std::pair<float, float>, 100> innert;
  innert.fill({100, -100});
  for (auto &c : handle_plane_inner_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.f, 1.f);

    if (100 * t.x < innermin) {
      innermin = 100 * t.x;
    }

    if (100 * t.x > innermax) {
      innermax = 100 * t.x;
    }

    innert[100 * t.x].first = std::min(t.y, innert[100 * t.x].first);
    innert[100 * t.x].second = std::max(t.y, innert[100 * t.x].second);
  }

  for (int timp = innermin; timp <= innermax; ++timp) {
    // cases
    if (innert[timp].first > 2.f)
      continue;
    // 1. both values the same
    if (timp > 94) {
      auto v = innert[timp].first;
      if (v <= handle_table[timp].first) {
        handle_table[timp] = {100.f, -100.f};
      } else if (handle_table[timp].second > 0.f) {
        handle_table[timp].second =
            std::min(std::clamp(v, 0.001f, 0.999f), handle_table[timp].second);
      }
    } // 2. values are different
    else {
      auto min = innert[timp].first;
      auto max = innert[timp].second;
      // handle min
      handle_table[timp].first =
          std::max(std::clamp(min, 0.001f, 0.999f), handle_table[timp].first);
      handle_table[timp].second =
          std::min(std::clamp(max, 0.001f, 0.999f), handle_table[timp].second);
    }
  }

  int i = 0;

  auto mix = [](float a, float b, float t) { return (1.f - t) * a + t * b; };

  bool started_even = ((i + 1) % 2) == 0;

  if (handle_table[i].first >= 0.f && handle_table[i].first <= 1.f &&
      handle_table[i].second >= 0.f && handle_table[i].second <= 1.f) {
    auto s = handle_sampler.sample(i / 99.f, handle_table[i].first);

    output << "N" << instruction++ << "G01"
           << "X" << s.x << "Y" << -s.z << std::endl;
  }

  for (; i < outermax; ++i) {
    if (handle_table[i].first > 1.f) {
      break;
    }
    float tx = i / 99.f;
    // first option - no intersection
    float start = std::min(handle_table[i].first, 0.999f);
    float end = std::max(handle_table[i].second, 0.001f);

    for (int j = 0; j < 20; ++j) {
      auto mix_val = std::clamp(j / 19.f, 0.001f, 0.999f);
      auto s = handle_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                             ? mix(start, end, mix_val)
                                             : mix(end, start, mix_val));

      if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
        throw;
      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << "Z" << 16.f + s.y - 4.f
             << std::endl;
    }
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 35.f << std::endl;

  i = innermin;
  while (handle_table[i].first > 1.f) {
    ++i;
  }

  started_even = ((i % 2) == 0);

  if (handle_table[i].first >= 0.f && handle_table[i].first <= 1.f &&
      handle_table[i].second >= 0.f && handle_table[i].second <= 1.f) {
    auto s = handle_sampler.sample(i / 99.f, handle_table[i].first);

    output << "N" << instruction++ << "G01"
           << "X" << s.x << "Y" << -s.z << std::endl;
  }

  for (; i < 99; ++i) {
    if (handle_table[i].first > 1.f) {
      continue;
    }
    float tx = i / 99.f;
    // first option - no intersection
    float start = std::min(handle_table[i].first, 0.999f);
    float end = std::max(handle_table[i].second, 0.001f);

    for (int j = 0; j < 20; ++j) {
      auto mix_val = std::clamp(j / 19.f, 0.001f, 0.999f);
      auto s = handle_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                             ? mix(start, end, mix_val)
                                             : mix(end, start, mix_val));

      if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
        throw;

      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << "Z" << 16.f + s.y - 4.f
             << std::endl;
    }
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;
}

void generate_hole_detail(std::ofstream &output, int &instruction) {
  auto &reg = ecs::registry::get_registry();
  // 1. generate intersection between main_body and the plane
  auto main_body_idx = find_named_c2_surface("main_body");
  auto handle_idx = find_named_c2_surface("handle");
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

  sampler tmp_handle_sampler = get_sampler(handle_idx);
  sampler handle_sampler = tmp_handle_sampler;
  handle_sampler.sample = [&](float u, float v) {
    return tmp_handle_sampler.sample(u, v) +
           4.f * tmp_handle_sampler.normal(u, v);
  };

  systems::intersection_params params{};
  auto &cursor_t = reg.get_component<transformation>(
                          reg.get_map<cursor_params>().begin()->first)
                       .translation;

  params.start_from_cursor = true;
  cursor_t = {36.917, 4.108, 10.137};
  auto plane_handle =
      systems::intersect(plane_idx, handle_idx, plane_sampler, handle_sampler,
                         params, false, cursor_t);

  params.start_from_cursor = false;
  params.subdivisions = 12;
  auto plane_main_body =
      systems::intersect(plane_idx, main_body_idx, plane_sampler,
                         main_body_sampler, params, false, cursor_t);

  params.start_from_cursor = true;
  cursor_t = {43.042, 5.005, 23.815};

  const auto plane_main_body_intersect = plane_main_body.idx;
  auto &plane_main_body_coords =
      reg.get_component<relationship>(plane_main_body_intersect)
          .virtual_children;

  const auto plane_handle_intersect = plane_handle.idx;
  auto &plane_handle_coords =
      reg.get_component<relationship>(plane_handle_intersect).virtual_children;

  // create a table of min/max
  // CAREFUL THIS ITERATES OVER V AND SLIDES OVER U,
  // NOT THE USUAL (REVERSED) WAY
  std::array<std::pair<float, float>, 100> hole_table;
  hole_table.fill({100.f, -100.f});

  int min_v =
      100 *
      reg.get_component<transformation>(plane_handle_coords[0]).translation.y;
  int u_val =
      reg.get_component<transformation>(plane_handle_coords[0]).translation.x;
  int max_v = 100 * reg.get_component<transformation>(
                           plane_handle_coords[plane_handle_coords.size() - 1])
                        .translation.y;

  for (auto &c : plane_handle_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.001f, 0.999f);
    t.y = std::clamp(t.y, 0.001f, 0.999f);
    hole_table[100 * t.y].first = std::min(hole_table[100 * t.y].first, t.x);
    hole_table[100 * t.y].second = std::max(hole_table[100 * t.y].second, t.x);
  }

  for (int tmp_v = min_v; tmp_v <= max_v; ++tmp_v) {
    hole_table[tmp_v].first = u_val;
  }

  for (auto &c : plane_main_body_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.001f, 0.999f);
    t.y = std::clamp(t.y, 0.001f, 0.999f);
    if (hole_table[100 * t.y].first <= 1.f) {
      hole_table[100 * t.y].first = std::max(hole_table[100 * t.y].first, t.x);
    }
  }

  int i = 0;

  auto mix = [](float a, float b, float t) { return (1.f - t) * a + t * b; };

  while (hole_table[i].first > 1.f) {
    ++i;
  }

  bool started_even = (i % 2) == 0;

  if (hole_table[i].first >= 0.f && hole_table[i].first <= 1.f &&
      hole_table[i].second >= 0.f && hole_table[i].second <= 1.f) {

    float start = std::max(hole_table[i].first, 0.01f);
    float end = std::min(hole_table[i].second, 0.99f);

    auto mix_val = std::clamp(0.f, 0.001f, 0.999f);
    auto s = plane_sampler.sample((started_even == ((i % 2) == 0))
                                      ? mix(start, end, mix_val)
                                      : mix(end, start, mix_val),
                                  i / 99.f);

    output << "N" << instruction++ << "G01"
           << "X" << s.x << "Y" << -s.z << std::endl;
  }

  while (i < 99 &&
         (hole_table[i + 1].first >= 0.f && hole_table[i + 1].first <= 1.f &&
          hole_table[i + 1].second >= 0.f && hole_table[i + 1].second <= 1.f)) {
    float ty = i / 99.f;
    // 1. find start position (not in hole)

    float start = std::max(hole_table[i].first, 0.01f);
    float end = std::min(hole_table[i].second, 0.99f);

    // go till hole
    for (int j = 0; j < 40; ++j) {
      auto mix_val = std::clamp(j / 39.f, 0.001f, 0.999f);
      auto s = plane_sampler.sample((started_even == ((i % 2) == 0))
                                        ? mix(start, end, mix_val)
                                        : mix(end, start, mix_val),
                                    ty);

      if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
        throw;
      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << "Z" << 16.f + s.y - 4.f
             << std::endl;
    }
    ++i;
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;
}

void finalize_intersections(std::ofstream &output, int &instruction) {
  auto &reg = ecs::registry::get_registry();

  std::vector<unsigned long> pi{
      find_named_point("obwodka1"), find_named_point("obwodka2"),
      find_named_point("obwodka3"), find_named_point("obwodka4"),
      find_named_point("obwodka5"), find_named_point("obwodka6"),
      find_named_point("obwodka7"), find_named_point("obwodka8"),
  };

  auto mill_between = [&](unsigned long p0, unsigned long p1) {
    std::vector<glm::vec3> points;
    for (unsigned long i = p0; i < p1; i = i + ((p0 < p1) ? 1 : -1)) {
      auto trans = reg.get_component<transformation>(i).translation;
      out(output, instruction++, trans.x, -trans.z, 12 + trans.y);
    }
  };

  // first pair
  auto trans1 = reg.get_component<transformation>(pi[0]).translation;
  output << "N" << instruction++ << "G01"
         << "X" << trans1.x << "Y" << -trans1.z << "Z" << 66.f << std::endl;
  mill_between(pi[0], pi[1]);
  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;
  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;
  // second pair
  auto trans2 = reg.get_component<transformation>(pi[2]).translation;
  output << "N" << instruction++ << "G01"
         << "X" << trans2.x << "Y" << -trans2.z << "Z" << 66.f << std::endl;
  mill_between(pi[2], pi[3]);
  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;
  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;
  // third pair
  auto trans3 = reg.get_component<transformation>(pi[4]).translation;
  output << "N" << instruction++ << "G01"
         << "X" << trans3.x << "Y" << -trans3.z << "Z" << 66.f << std::endl;
  mill_between(pi[4], pi[5]);
  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;
  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;
  // fourth pair
  auto trans4 = reg.get_component<transformation>(pi[6]).translation;
  output << "N" << instruction++ << "G01"
         << "X" << trans4.x << "Y" << -trans4.z << "Z" << 66.f << std::endl;
  mill_between(pi[6], pi[7]);
  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;
  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;
}

void generate_top_detail(std::ofstream &output, int &instruction) {
  auto &reg = ecs::registry::get_registry();
  // 1. generate intersection between main_body and the plane
  auto main_body_idx = find_named_c2_surface("main_body");
  auto top_idx = find_named_c2_surface("top");
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

  sampler tmp_top_sampler = get_sampler(top_idx);
  sampler top_sampler = tmp_top_sampler;
  top_sampler.sample = [&](float u, float v) {
    return tmp_top_sampler.sample(u, v) + 4.f * tmp_top_sampler.normal(u, v);
  };

  sampler top_sampler_blade = tmp_top_sampler;
  top_sampler_blade.sample = [&](float u, float v) {
    const static glm::vec3 center =
        glm::vec3{22.823f + -3.229f, 0.f, -50.611f + -50.106f} / 2.f;

    const auto s = tmp_top_sampler.sample(u, v);
    const auto norm = glm::normalize(s - center);
    return s + 4.2f * norm;
  };

  systems::intersection_params params{};
  auto &cursor_t = reg.get_component<transformation>(
                          reg.get_map<cursor_params>().begin()->first)
                       .translation;
  params.start_from_cursor = true;

  cursor_t = {-4.526, 3.988, -43.332};
  auto top_plane_left_bottom = systems::intersect(
      top_idx, plane_idx, top_sampler, plane_sampler, params, false, cursor_t);

  cursor_t = {-1.859, 3.954, -57.094};
  auto top_plane_left_top = systems::intersect(
      top_idx, plane_idx, top_sampler, plane_sampler, params, false, cursor_t);

  cursor_t = {20.802, 4.043, -57.624};
  auto top_plane_right_top = systems::intersect(
      top_idx, plane_idx, top_sampler, plane_sampler, params, false, cursor_t);

  cursor_t = {21.855, 4.017, -32.862};
  auto top_plane_right_bottom = systems::intersect(
      top_idx, plane_idx, top_sampler, plane_sampler, params, false, cursor_t);

  auto top_main_body =
      systems::intersect(top_idx, main_body_idx, top_sampler, main_body_sampler,
                         {}, false, cursor_t);

  const auto top_main_body_intersect = top_main_body.idx;
  auto &top_main_body_coords =
      reg.get_component<relationship>(top_main_body_intersect).virtual_children;

  const auto top_plane_left_bottom_intersect = top_plane_left_bottom.idx;
  auto &top_plane_left_bottom_coords =
      reg.get_component<relationship>(top_plane_left_bottom_intersect)
          .virtual_children;

  const auto top_plane_left_top_intersect = top_plane_left_top.idx;
  auto &top_plane_left_top_coords =
      reg.get_component<relationship>(top_plane_left_top_intersect)
          .virtual_children;

  const auto top_plane_right_top_intersect = top_plane_right_top.idx;
  auto &top_plane_right_top_coords =
      reg.get_component<relationship>(top_plane_right_top_intersect)
          .virtual_children;

  const auto top_plane_right_bottom_intersect = top_plane_right_bottom.idx;
  auto &top_plane_right_bottom_coords =
      reg.get_component<relationship>(top_plane_right_bottom_intersect)
          .virtual_children;

  // create a table of min/max
  std::array<std::pair<float, float>, 100> top_table;
  top_table.fill({0.2f, 0.999f});

  for (auto &c : top_main_body_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.001f, 0.999f);
    top_table[100 * t.x].second =
        std::min(top_table[100 * t.x].second, std::clamp(t.y, 0.02f, 0.999f));
  }

  int rightbmin = 101, rightbmax = -1;
  int rightbymin = 101, rightbymax = -1;
  for (auto &c : top_plane_right_bottom_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.001f, 0.999f);

    if (100 * t.x < rightbmin) {
      rightbmin = 100 * t.x;
    }

    if (100 * t.x > rightbmax) {
      rightbmax = 100 * t.x;
    }

    if (100 * t.y < rightbymin) {
      rightbymin = 100 * t.y;
    }

    if (100 * t.y > rightbymax) {
      rightbymax = 100 * t.y;
    }

    if (t.y >= top_table[100 * t.x].second) {
      top_table[100 * t.x] = {100.f, -100.f};
    } else {
      top_table[100 * t.x].first =
          std::max(top_table[100 * t.x].first, std::clamp(t.y, 0.02f, 1.f));
    }
  }

  int righttmin = 101, righttmax = -1;
  int righttymin = 101, righttymax = -1;
  for (auto &c : top_plane_right_top_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.001f, 0.999f);

    if (100 * t.x < righttmin) {
      righttmin = 100 * t.x;
    }

    if (100 * t.x > righttmax) {
      righttmax = 100 * t.x;
    }

    if (100 * t.y < righttymin) {
      righttymin = 100 * t.y;
    }

    if (100 * t.y > righttymax) {
      righttymax = 100 * t.y;
    }

    if (t.y >= top_table[100 * t.x].second) {
      top_table[100 * t.x] = {100.f, -100.f};
    } else {
      top_table[100 * t.x].first =
          std::max(top_table[100 * t.x].first, std::clamp(t.y, 0.02f, 1.f));
    }
  }

  int leftbmin = 101, leftbmax = -1;
  int leftbymin = 101, leftbymax = -1;
  for (auto &c : top_plane_left_bottom_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.02f, 0.999f);

    if (100 * t.x < leftbmin) {
      leftbmin = 100 * t.x;
    }

    if (100 * t.x > leftbmax) {
      leftbmax = 100 * t.x;
    }

    if (100 * t.y < leftbymin) {
      leftbymin = 100 * t.y;
    }

    if (100 * t.y > leftbymax) {
      leftbymax = 100 * t.y;
    }

    if (t.y >= top_table[100 * t.x].second) {
      top_table[100 * t.x] = {100.f, -100.f};
    } else {
      top_table[100 * t.x].first =
          std::max(top_table[100 * t.x].first, std::clamp(t.y, 0.02f, 1.f));
    }
  }

  int lefttmin = 101, lefttmax = -1;
  int lefttymin = 101, lefttymax = -1;
  for (auto &c : top_plane_left_top_coords) {
    auto &t = reg.get_component<transformation>(c).translation;
    t.x = std::clamp(t.x, 0.02f, 0.999f);

    if (100 * t.x < lefttmin) {
      lefttmin = 100 * t.x;
    }

    if (100 * t.x > lefttmax) {
      lefttmax = 100 * t.x;
    }

    if (100 * t.y < lefttymin) {
      lefttymin = 100 * t.y;
    }

    if (100 * t.y > lefttymax) {
      lefttymax = 100 * t.y;
    }

    if (t.y >= top_table[100 * t.x].second) {
      top_table[100 * t.x] = {100.f, -100.f};
    } else {
      top_table[100 * t.x].first =
          std::max(top_table[100 * t.x].first, std::clamp(t.y, 0.02f, 1.f));
    }
  }

  // and now 0.f -> 0.5f
  auto start_blade_u = 0.f;
  auto end_blade_u = 0.f;

  float curr_dist = 100.f;
  for (int blade = 0; blade < 100; ++blade) {
    float bladef = blade / 99.f;
    auto s = top_sampler.sample(bladef, 0.5f);
    if (blade == 50) {
      curr_dist = 100.f;
    }
    if (std::abs(s.y - 4.f) < curr_dist) {
      if (blade < 50) {
        start_blade_u = bladef;
      } else
        end_blade_u = bladef;
    }
    curr_dist = std::abs(s.y - 4.f);
  }

  auto mix = [](float a, float b, float t) { return (1.f - t) * a + t * b; };

  {
    auto s = top_sampler_blade.sample(start_blade_u, 0.5f);

    output << "N" << instruction++ << "G01"
           << "X" << s.x << "Y" << -s.z << std::endl;
  }

  const auto dist = (1.f - end_blade_u) + start_blade_u;
  for (int j = 0; j < 40; ++j) {

    auto uc = start_blade_u - (j / 39.f) * dist;
    if (uc < 0.f) {
      uc += 1.f;
    }
    auto s = top_sampler_blade.sample(uc, 0.5f);

    output << "N" << instruction++ << "G01"
           << "X" << s.x << "Y" << -s.z << "Z" << 16.f + s.y - 4.f << std::endl;
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 35.f << std::endl;

  int i = 0;

  bool started_even = (i % 2) == 0;

  if (top_table[i].first >= 0.f && top_table[i].first <= 1.f &&
      top_table[i].second >= 0.f && top_table[i].second <= 1.f) {
    auto s = top_sampler.sample(i / 99.f, top_table[i].first);

    output << "N" << instruction++ << "G01"
           << "X" << s.x << "Y" << -s.z << std::endl;
  }

  for (; i < rightbmax; ++i) {
    if (static_cast<int>(100 * top_table[i].first) >= 50) {
      break;
    }
    float tx = std::clamp(i / 99.f, 0.02f, 0.99f);
    // first option - no intersection
    float start = std::min(top_table[i].first, 0.99f);
    float end = std::min(0.49f, std::max(top_table[i].second, 0.01f));

    for (int j = 0; j < 20; ++j) {
      auto mix_val = std::clamp(j / 19.f, 0.001f, 0.999f);
      auto s = top_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                          ? mix(start, end, mix_val)
                                          : mix(end, start, mix_val));

      if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
        throw;
      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << "Z" << 16.f + s.y - 4.f
             << std::endl;
    }
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 35.f << std::endl;

  i = leftbmin;
  while (top_table[i].first > 0.5f) {
    ++i;
  }

  if (top_table[i].first >= 0.f && top_table[i].first <= 1.f &&
      top_table[i].second >= 0.f && top_table[i].second <= 1.f) {
    auto s = top_sampler.sample(i / 99.f, top_table[i].first);

    output << "N" << instruction++ << "G01"
           << "X" << s.x << "Y" << -s.z << std::endl;
  }

  started_even = (i % 2) == 0;

  for (; i < 100; ++i) {
    if (static_cast<int>(100 * top_table[i].first) >= 50) {
      continue;
    }
    float tx = i / 99.f;
    // first option - no intersection
    float start = std::min(top_table[i].first, 0.999f);
    float end = std::min(0.49f, std::max(top_table[i].second, 0.001f));

    for (int j = 0; j < 20; ++j) {
      auto mix_val = std::clamp(j / 19.f, 0.001f, 0.999f);
      auto s = top_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                          ? mix(start, end, mix_val)
                                          : mix(end, start, mix_val));

      if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
        throw;
      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << "Z" << 16.f + s.y - 4.f
             << std::endl;
    }
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;
  // first do it for 0.5f - 1.f
  i = 0;

  started_even = (i % 2) == 0;

  if (top_table[i].first >= 0.f && top_table[i].first <= 1.f &&
      top_table[i].second >= 0.f && top_table[i].second <= 1.f) {
    auto s = top_sampler.sample(i / 99.f, top_table[i].first);

    output << "N" << instruction++ << "G01"
           << "X" << s.x << "Y" << -s.z << std::endl;
  }

  for (; i < rightbmax; ++i) {
    if (top_table[i].first > 1.f) {
      break;
    }
    float tx = i / 99.f;
    // first option - no intersection
    float start = std::max(0.51f, std::min(top_table[i].first, 0.999f));
    float end = std::max(top_table[i].second, 0.001f);

    for (int j = 0; j < 20; ++j) {
      auto mix_val = std::clamp(j / 19.f, 0.001f, 0.999f);
      auto s = top_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                          ? mix(start, end, mix_val)
                                          : mix(end, start, mix_val));

      if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
        throw;
      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << "Z" << 16.f + s.y - 4.f
             << std::endl;
    }
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 35.f << std::endl;

  i = leftbmin;
  while (top_table[i].first > 1.f) {
    ++i;
  }

  if (top_table[i].first >= 0.f && top_table[i].first <= 1.f &&
      top_table[i].second >= 0.f && top_table[i].second <= 1.f) {
    auto s = top_sampler.sample(i / 99.f, top_table[i].first);

    output << "N" << instruction++ << "G01"
           << "X" << s.x << "Y" << -s.z << std::endl;
  }

  for (; i < 100; ++i) {
    if (top_table[i].first > 1.f) {
      continue;
    }
    float tx = i / 99.f;
    // first option - no intersection
    float start = std::max(0.51f, std::min(top_table[i].first, 0.999f));
    float end = std::max(top_table[i].second, 0.001f);

    for (int j = 0; j < 20; ++j) {
      auto mix_val = std::clamp(j / 19.f, 0.001f, 0.999f);
      auto s = top_sampler.sample(tx, (started_even == ((i % 2) == 0))
                                          ? mix(start, end, mix_val)
                                          : mix(end, start, mix_val));

      if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
        throw;
      output << "N" << instruction++ << "G01"
             << "X" << s.x << "Y" << -s.z << "Z" << 16.f + s.y - 4.f
             << std::endl;
    }
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;
}

void generate_third_stage(std::filesystem::path path) {
  //  R is 4.f
  std::ofstream output;

  output.setf(std::ios::fixed, std::ios::floatfield);
  output.precision(3);

  output.open(path);
  int instruction = 1;
  float x{0}, y{0}, z{66};

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << 66.f << std::endl;

  x = 0.f;
  y = 0.f;

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << z << std::endl;

  generate_top_detail(output, instruction);
  generate_main_body_detail(output, instruction);
  generate_front_detail(output, instruction);
  generate_hole_detail(output, instruction);
  generate_handle_detail(output, instruction);
  finalize_intersections(output, instruction);

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;
  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;

  output.close();
}

void generate_fourth_stage(std::filesystem::path path) {
  //  R is 1.f
  std::ofstream output;

  output.setf(std::ios::fixed, std::ios::floatfield);
  output.precision(3);

  output.open(path);
  int instruction = 1;
  float x{0}, y{0}, z{66.f};

  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << "Z" << 66.f << std::endl;

  x = 0.f;
  y = 0.f;

  output << "N" << instruction++ << "G01"
         << "X" << x << "Y" << y << "Z" << z << std::endl;

  // UKROP here
  std::vector<glm::vec3> u_points{{-50.f, -45.f, 15.f},
                                  {-50.f, -65.f, 15.f},
                                  {-34.f, -65.f, 15.f},
                                  {-34.f, -45.f, 15.f}};

  std::vector<glm::vec3> k_points{
      {-29.f, -45.f, 15.f}, {-29.f, -57.f, 15.f},
      {-13.f, -45.f, 15.f}, {0.f, 0.f, 15.f}, // middle
      {-13.f, -65.f, 15.f}, {0.f, 0.f, 15.f}, // middle
      {-29.f, -57.f, 15.f}, {-29.f, -65.f, 15.f}};

  k_points[5] = k_points[3] = glm::mix(k_points[1], k_points[2], 0.45);

  std::vector<glm::vec3> r_points{{-8.f, -65.f, 15.f}, {-8.f, -45.f, 15.f},
                                  {8.f, -45.f, 15.f},  {8.f, -55.f, 15.f},
                                  {-8.f, -55.f, 15.f}, {8.f, -65.f, 15.f}};
  std::vector<glm::vec3> o_points{{13.f, -45.f, 15.f},
                                  {13.f, -65.f, 15.f},
                                  {29.f, -65.f, 15.f},
                                  {29.f, -45.f, 15.f},
                                  {13.f, -45.f, 15.f}};

  std::vector<glm::vec3> p_points{{34.f, -65.f, 15.f},
                                  {34.f, -45.f, 15.f},
                                  {50.f, -45.f, 15.f},
                                  {50.f, -55.f, 15.f},
                                  {34.f, -55.f, 15.f}};

  // U
  output << "N" << instruction++ << "G01"
         << "X" << u_points[0].x << "Y" << u_points[0].y << std::endl;

  for (auto &p : u_points) {
    output << "N" << instruction++ << "G01"
           << "X" << p.x << "Y" << p.y << "Z" << p.z << std::endl;
  }

  // K
  output << "N" << instruction++ << "G01"
         << "Z" << 20.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << k_points[0].x << "Y" << k_points[0].y << std::endl;

  for (auto &p : k_points) {
    output << "N" << instruction++ << "G01"
           << "X" << p.x << "Y" << p.y << "Z" << p.z << std::endl;
  }
  // R
  output << "N" << instruction++ << "G01"
         << "Z" << 20.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << r_points[0].x << "Y" << r_points[0].y << std::endl;

  for (auto &p : r_points) {
    output << "N" << instruction++ << "G01"
           << "X" << p.x << "Y" << p.y << "Z" << p.z << std::endl;
  }

  // O
  output << "N" << instruction++ << "G01"
         << "Z" << 20.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << o_points[0].x << "Y" << o_points[0].y << std::endl;

  for (auto &p : o_points) {
    output << "N" << instruction++ << "G01"
           << "X" << p.x << "Y" << p.y << "Z" << p.z << std::endl;
  }

  // P
  output << "N" << instruction++ << "G01"
         << "Z" << 20.f << std::endl;

  output << "N" << instruction++ << "G01"
         << "X" << p_points[0].x << "Y" << p_points[0].y << std::endl;

  for (auto &p : p_points) {
    output << "N" << instruction++ << "G01"
           << "X" << p.x << "Y" << p.y << "Z" << p.z << std::endl;
  }

  output << "N" << instruction++ << "G01"
         << "Z" << 66.f << std::endl;
  output << "N" << instruction++ << "G01"
         << "X" << 0.f << "Y" << 0.f << std::endl;

  output.close();
}
} // namespace paths
