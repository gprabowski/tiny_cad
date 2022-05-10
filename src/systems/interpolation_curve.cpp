#include <systems.h>

#include <array>
#include <set>

namespace systems {

void regenerate_icurve(ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();

  relationship &_r = reg.get_component<relationship>(idx);
  gl_object &g = reg.get_component<gl_object>(idx);
  std::vector<glm::vec4> &out_vertices = g.points;
  std::vector<unsigned int> &out_indices = g.indices;

  auto &transformations = reg.get_map<transformation>();
  std::vector<ecs::EntityType> children;

  glm::vec3 last =
      transformations[_r.children[0]].translation - glm::vec3{0.f, 1.f, 1.f};

  for (const auto c : _r.children) {
    auto &_p = transformations[c].translation;
    if (glm::length(_p - last) > 1e-4) {
      last = _p;
      children.push_back(c);
    }
  }
  out_vertices.clear();
  out_indices.clear();

  const auto N = children.size() - 1;

  // get distances
  std::vector<float> di;
  for (std::size_t i = 0; i < children.size() - 1; ++i) {
    auto &t1 = transformations[children[i]].translation;
    auto &t2 = transformations[children[i + 1]].translation;
    di.push_back(glm::length(t2 - t1));
  }

  // generate the initial matrix
  std::vector<std::array<float, 3>> diags(N);
  const auto alfa = [&](int i) { return di[i - 1] / (di[i - 1] + di[i]); };
  const auto beta = [&](int i) { return di[i] / (di[i - 1] + di[i]); };

  for (std::size_t i = 1; i <= N - 1; ++i) {
    diags[i] = {alfa(i), 2.f, beta(i)};
  }

  // generate RHS
  std::vector<glm::vec3> rhs(N);
  for (std::size_t i = 1; i < N; ++i) {
    auto &t1 = transformations[children[i - 1]].translation;
    auto &t2 = transformations[children[i]].translation;
    auto &t3 = transformations[children[i + 1]].translation;

    rhs[i] =
        3.f * ((t3 - t2) / di[i] - (t2 - t1) / di[i - 1]) / (di[i - 1] + di[i]);
  }

  // prepare output data structures
  std::vector<glm::vec3> a(N + 1);
  std::vector<glm::vec3> b(N + 1);
  std::vector<glm::vec3> c(N + 1);
  std::vector<glm::vec3> d(N + 1);

  // solve matrix for c
  // space and time linear algorithm from
  // https://people.sc.fsu.edu/~jburkardt/classes/math2071_2020/tridiagonal/tridiagonal.pdf
  for (std::size_t i = 1; i < N - 1; ++i) {
    if (fabs(diags[i][1]) < 1e-4) {
      throw "failed interpolation";
    }
    const auto s = diags[i + 1][0] / diags[i][1];
    diags[i + 1][1] -= s * diags[i][2];
    diags[i + 1][0] = 0.f;
    rhs[i + 1] -= s * rhs[i];
  }

  c[0] = glm::vec3(0.0f);
  c[N] = glm::vec3(0.0f);
  c[N - 1] = rhs[N - 1] / diags[N - 1][1];

  for (int i = N - 2; i > 0; --i) {
    c[i] = (rhs[i] - diags[i][2] * c[i + 1]) / diags[i][1];
  }

  // get d from c
  for (int i = N - 1; i >= 0; --i) {
    d[i] = (c[i + 1] - c[i]) / (3.f * di[i]);
  }

  // get a
  for (int i = N - 1; i >= 0; --i) {
    auto &t = transformations[children[i]].translation;
    a[i] = t;
  }

  // get b from c, d
  for (int i = N - 1; i >= 0; --i) {
    auto &t = transformations[children[i + 1]].translation;
    b[i] = (t - a[i] - c[i] * di[i] * di[i] - d[i] * di[i] * di[i] * di[i]) /
           di[i];
  }
  // generate points by multiplying by the base change matrix
  // clang-format off
  glm::mat4 to_bezier{{1.f, 1.f, 1.f, 1.f},   
                      {0, 1.f/3.f, 2.f/3.f, 1.f},
                      {0.f, 0.f, 1.f / 3.f, 1.f}, 
                      {0.f, 0.f, 0.f, 1.f}};

  std::array<glm::vec4, 3> bezier_p;
  for(std::size_t i = 0; i < N; ++i) {
    for(int j = 0; j < 3; ++j) {
      bezier_p[j] = to_bezier * glm::vec4(a[i][j], b[i][j] * di[i], c[i][j] * di[i] * di[i], d[i][j] * di[i] * di[i] * di[i]);
    }
    out_vertices.push_back(glm::vec4(bezier_p[0][0], bezier_p[1][0], bezier_p[2][0], 1.0f));
    out_vertices.push_back(glm::vec4(bezier_p[0][1], bezier_p[1][1], bezier_p[2][1], 1.0f));
    out_vertices.push_back(glm::vec4(bezier_p[0][2], bezier_p[1][2], bezier_p[2][2], 1.0f));
    out_vertices.push_back(glm::vec4(bezier_p[0][3], bezier_p[1][3], bezier_p[2][3], 1.0f));
  }

  for (std::size_t i = 0; i < out_vertices.size(); ++i) {
    out_indices.push_back(i);
  }

  reset_gl_objects(g);
}
} // namespace systems
