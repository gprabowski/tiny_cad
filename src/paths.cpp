#include <paths.hpp>

#include <fstream>

namespace paths {
void generate_first_stage(float *data, int resolution, int size,
                          std::filesystem::path path) {
  float halfsize = size / 2.f;
  int num_steps = 10;
  float step = halfsize / num_steps;
  float curr_step = step;
  int instruction = 1;

  std::ofstream output;
  output.open(path);

  auto sample = [&](float fx, float fy) {
    const int x = ((fx / size) + 0.5f) * resolution;
    const int y = ((fy / size) + 0.5f) * resolution;

    return 5.f + data[y * resolution + x];
  };

  float x{-halfsize}, y{-halfsize}, z{data[0]};

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
             << "X" << x << "Y" << y << "Z" << sample(x, y) << std::endl;
      y += step;
      output << "N" << instruction++ << "G01"
             << "X" << x << "Y" << y << "Z" << sample(x, y) << std::endl;
      curr_step = curr_step * -1;
      continue;
    }
    output << "N" << instruction++ << "G01"
           << "X" << x << "Y" << y << "Z" << sample(x, y) << std::endl;
  }

  output.close();
}
} // namespace paths
