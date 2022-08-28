#include <registry.h>

#include "imgui.h"
#include "imgui_internal.h"
#include <misc/cpp/imgui_stdlib.h>

#include <implot/implot.h>

#include <ImGuiFileDialog.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <utility>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <ImGuizmo.h>

#include <Serializer.h>
#include <constant_matrices.h>
#include <constructors.h>
#include <frame_state.h>
#include <frame_update.h>
#include <framebuffer.h>
#include <gl_object.h>
#include <gui.h>
#include <systems.h>
#include <torus.h>

namespace gui {

template <class Enum>
constexpr std::underlying_type_t<Enum> to_underlying(Enum e) noexcept {
  return static_cast<std::underlying_type_t<Enum>>(e);
}

static auto vector_getter = [](void *vec, int idx, const char **out_text) {
  auto &vector = *static_cast<std::vector<std::string> *>(vec);
  if (idx < 0 || idx >= static_cast<int>(vector.size())) {
    return false;
  }
  *out_text = vector.at(idx).c_str();
  return true;
};

// color theme copied from thecherno/hazel
void set_dark_theme() {
  auto &colors = ImGui::GetStyle().Colors;
  colors[ImGuiCol_WindowBg] = ImVec4{0.1f, 0.105f, 0.11f, 1.0f};

  // Headers
  colors[ImGuiCol_Header] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};
  colors[ImGuiCol_HeaderHovered] = ImVec4{0.3f, 0.305f, 0.31f, 1.0f};
  colors[ImGuiCol_HeaderActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};

  // Buttons
  colors[ImGuiCol_Button] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};
  colors[ImGuiCol_ButtonHovered] = ImVec4{0.3f, 0.305f, 0.31f, 1.0f};
  colors[ImGuiCol_ButtonActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};

  // Frame BG
  colors[ImGuiCol_FrameBg] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};
  colors[ImGuiCol_FrameBgHovered] = ImVec4{0.3f, 0.305f, 0.31f, 1.0f};
  colors[ImGuiCol_FrameBgActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};

  // Tabs
  colors[ImGuiCol_Tab] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TabHovered] = ImVec4{0.38f, 0.3805f, 0.381f, 1.0f};
  colors[ImGuiCol_TabActive] = ImVec4{0.28f, 0.2805f, 0.281f, 1.0f};
  colors[ImGuiCol_TabUnfocused] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TabUnfocusedActive] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};

  // Title
  colors[ImGuiCol_TitleBg] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TitleBgActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TitleBgCollapsed] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
}

void setup_gui(std::shared_ptr<GLFWwindow> &w) {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
  // io.ConfigViewportsNoAutoMerge = true;
  // io.ConfigViewportsNoTaskBarIcon = true;

  ImGui::StyleColorsDark();

  // when viewports are enables we tweak WindowRounding/WIndowBg so platform
  // windows can look identical to regular ones
  ImGuiStyle &style = ImGui::GetStyle();
  if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    style.WindowRounding = 0.0f;
    style.Colors[ImGuiCol_WindowBg].w = 1.0f;
  }

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(w.get(), true);
  ImGui_ImplOpenGL3_Init("#version 460");

  // fonts
  io.FontDefault = io.Fonts->AddFontFromFileTTF(
      //"fonts/opensans/static/OpenSans/OpenSans-Regular.ttf",
      "fonts/jbmono/fonts/ttf/JetBrainsMono-Regular.ttf", 18.0f);
  set_dark_theme();
}

void cleanup_gui() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
}

// utility structure for realtime plot
struct ScrollingBuffer {
  int MaxSize;
  int Offset;
  ImVector<ImVec2> Data;
  ScrollingBuffer(int max_size = 2000) {
    MaxSize = max_size;
    Offset = 0;
    Data.reserve(MaxSize);
  }
  void AddPoint(float x, float y) {
    if (Data.size() < MaxSize)
      Data.push_back(ImVec2(x, y));
    else {
      Data[Offset] = ImVec2(x, y);
      Offset = (Offset + 1) % MaxSize;
    }
  }
  void Erase() {
    if (Data.size() > 0) {
      Data.shrink(0);
      Offset = 0;
    }
  }
};

// utility structure for realtime plot
struct RollingBuffer {
  float Span;
  ImVector<ImVec2> Data;
  RollingBuffer() {
    Span = 10.0f;
    Data.reserve(2000);
  }
  void AddPoint(float x, float y) {
    float xmod = fmodf(x, Span);
    if (!Data.empty() && xmod < Data.back().x)
      Data.shrink(0);
    Data.push_back(ImVec2(xmod, y));
  }
};

void ShowDemo_RealtimePlots() {
  static ScrollingBuffer sdata1;
  static RollingBuffer rdata1;
  static float t = 0;
  t += ImGui::GetIO().DeltaTime;
  sdata1.AddPoint(t, frame_state::last_cpu_frame);
  rdata1.AddPoint(t, ImGui::GetIO().Framerate);

  static float history = 10.0f;
  ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");
  rdata1.Span = history;

  static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

  if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, 150))) {
    ImPlot::SetupAxes(NULL, NULL, flags, flags);
    ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 16.f);
    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
    ImPlot::PlotShaded("Frame time in ms", &sdata1.Data[0].x, &sdata1.Data[0].y,
                       sdata1.Data.size(), -INFINITY, sdata1.Offset,
                       2 * sizeof(float));
    ImPlot::EndPlot();
  }
  if (ImPlot::BeginPlot("##Rolling", ImVec2(-1, 150))) {
    ImPlot::SetupAxes(NULL, NULL, flags, flags);
    ImPlot::SetupAxisLimits(ImAxis_X1, 0, history, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 100);
    ImPlot::PlotLine("FPS", &rdata1.Data[0].x, &rdata1.Data[0].y,
                     rdata1.Data.size(), 0, 2 * sizeof(float));
    ImPlot::EndPlot();
  }
}

void start_frame() {
  static bool show_demo = false;
  ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode;
  ImGuiWindowFlags host_window_flags = 0;
  host_window_flags |= ImGuiWindowFlags_NoTitleBar |
                       ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                       ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking;
  host_window_flags |=
      ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
  if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
    host_window_flags |= ImGuiWindowFlags_NoBackground;

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();

  ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(), dockspace_flags);

  if (show_demo) {
    ImGui::ShowDemoWindow(&show_demo);
    ImPlot::ShowDemoWindow();
  }
}

void render_performance_window() {
  ImGui::Begin("Frame Statistics");
  ShowDemo_RealtimePlots();
  ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
  ImGui::Text("Last CPU frame %.3lf ms", frame_state::last_cpu_frame);
  ImGui::End();
}

void render_general_settings() {
  // show general settings window
  auto &s = input_state::get_input_state();
  auto &fb = framebuffer::get();

  static std::vector<std::string> wasd_values{"Camera Movement",
                                              "Cursor Movement"};

  static std::vector<std::string> gizmo_values{"Translation", "Rotation",
                                               "Scaling", "Universal"};

  static std::vector<std::string> anaglyph_values{
      "mono",           "true anaglyph",       "gray anaglyph",
      "color anaglyph", "half color anaglyph", "optimized anaglyph",
      "custom anaglyph"};

  ImGui::Begin("General Settings");

  static input_state::gizmo_mode gmode{input_state::gizmo_mode::universal};

  ImGui::Text("Controls");
  if (ImGui::Combo("wasd mode", reinterpret_cast<int *>(&s.imode),
                   vector_getter, static_cast<void *>(&wasd_values),
                   wasd_values.size())) {
    // reaction for input mode change
  }

  if (ImGui::Combo("gizmo mode", reinterpret_cast<int *>(&gmode), vector_getter,
                   static_cast<void *>(&gizmo_values), gizmo_values.size())) {
    // reaction for input mode change
    switch (gmode) {
    case input_state::gizmo_mode::translation: {
      s.gizmo_op = ImGuizmo::OPERATION::TRANSLATE;
    } break;
    case input_state::gizmo_mode::rotation: {
      s.gizmo_op = ImGuizmo::OPERATION::ROTATE;
    } break;
    case input_state::gizmo_mode::scaling: {
      s.gizmo_op = ImGuizmo::OPERATION::SCALE;
    } break;
    case input_state::gizmo_mode::universal: {
      s.gizmo_op = ImGuizmo::OPERATION::UNIVERSAL;
    }
    }
  }

  ImGui::Text("Rendering");
  ImGui::SliderFloat("Near Clip", &s.clip_near, 0.00001f, 1.0f);
  ImGui::SliderFloat("Far Clip", &s.clip_near, s.clip_near + 0.00001f,
                     10000.0f);
  ImGui::SliderFloat("FOV Y", &s.fov_y, 0, 90);

  ImGui::Text(" ");
  ImGui::End();

  ImGui::Begin("Camera Settings");
  if (ImGui::SliderFloat3("position", glm::value_ptr(s.cam_pos), -1000.f,
                          1000.f)) {
  }
  if (ImGui::SliderFloat3("front", glm::value_ptr(s.cam_front), -1000.f,
                          1000.f)) {
  }
  ImGui::End();

  ImGui::Begin("Anaglyphs");
  ImGui::SliderFloat("Convergence", &s.ster_sett.convergence, s.clip_near,
                     s.clip_far);
  ImGui::SliderFloat("Eye Distance", &s.ster_sett.eye_separation, 0.001f,
                     100.f);
  if (ImGui::Combo("anaglyph mode", reinterpret_cast<int *>(&s.ster_sett.mode),
                   vector_getter, static_cast<void *>(&anaglyph_values),
                   anaglyph_values.size())) {
    // reaction for input mode change
    switch (s.ster_sett.mode) {
    case stereo_settings::stereo_mode::mono: {
    } break;
    case stereo_settings::stereo_mode::true_anaglyph: {
      fb.setup_stereo_textures();
      frame_state::col_mat_left = constant_matrices::anaglyph_true_left;
      frame_state::col_mat_right = constant_matrices::anaglyph_true_right;
    } break;
    case stereo_settings::stereo_mode::gray_anaglyph: {
      fb.setup_stereo_textures();
      frame_state::col_mat_left = constant_matrices::anaglyph_gray_left;
      frame_state::col_mat_right = constant_matrices::anaglyph_gray_right;
    } break;
    case stereo_settings::stereo_mode::color_anaglyph: {
      fb.setup_stereo_textures();
      frame_state::col_mat_left = constant_matrices::anaglyph_color_left;
      frame_state::col_mat_right = constant_matrices::anaglyph_color_right;
    } break;
    case stereo_settings::stereo_mode::half_color_anaglyph: {
      fb.setup_stereo_textures();
      frame_state::col_mat_left = constant_matrices::anaglyph_half_color_left;
      frame_state::col_mat_right = constant_matrices::anaglyph_half_color_right;
    } break;
    case stereo_settings::stereo_mode::optimized_anaglyph: {
      fb.setup_stereo_textures();
      frame_state::col_mat_left = constant_matrices::anaglyph_optimized_left;
      frame_state::col_mat_right = constant_matrices::anaglyph_optimized_right;
    } break;
    case stereo_settings::stereo_mode::custom_anaglyph: {
      fb.setup_stereo_textures();
    } break;
    }
  }
  if (s.ster_sett.mode == stereo_settings::stereo_mode::custom_anaglyph) {
    bool modified{false};
    glm::vec3 lr = glm::row(frame_state::col_mat_left, 0),
              lg = glm::row(frame_state::col_mat_left, 1),
              lb = glm::row(frame_state::col_mat_left, 2),
              rr = glm::row(frame_state::col_mat_right, 0),
              rg = glm::row(frame_state::col_mat_right, 1),
              rb = glm::row(frame_state::col_mat_right, 2);

    ImGui::Text("Left eye");
    if (ImGui::SliderFloat3("Red##l", glm::value_ptr(lr), -1.0f, 1.0f)) {
      modified = true;
    }
    if (ImGui::SliderFloat3("Green##l", glm::value_ptr(lg), -1.0f, 1.0f)) {
      modified = true;
    }
    if (ImGui::SliderFloat3("Blue##l", glm::value_ptr(lb), -1.0f, 1.0f)) {
      modified = true;
    }

    ImGui::Text("Right eye");
    if (ImGui::SliderFloat3("Red##r", glm::value_ptr(rr), -1.0f, 1.0f)) {
      modified = true;
    }
    if (ImGui::SliderFloat3("Green##r", glm::value_ptr(rg), -1.0f, 1.0f)) {
      modified = true;
    }
    if (ImGui::SliderFloat3("Blue##r", glm::value_ptr(rb), -1.0f, 1.0f)) {
      modified = true;
    }

    if (modified) {
      frame_state::col_mat_left = glm::mat4{{lr.x, lg.x, lb.x, 0.f},
                                            {lr.y, lg.y, lb.y, 0.f},
                                            {lr.z, lg.z, lb.z, 0.f},
                                            {0.f, 0.f, 0.f, 1.f}};

      frame_state::col_mat_right = glm::mat4{{rr.x, rg.x, rb.x, 0.f},
                                             {rr.y, rg.y, rb.y, 0.f},
                                             {rr.z, rg.z, rb.z, 0.f},
                                             {0.f, 0.f, 0.f, 1.f}};
    }
  }
  ImGui::Text(" ");
  ImGui::End();
}

void render_figure_gui(ecs::EntityType idx, tag_figure &t,
                       const std::function<void(void)> &render_func) {
  std::string tree_id = t.name + ("##") + std::to_string(idx);
  ImGui::Text("%s", t.name.c_str());

  std::string desc = ("Show more##") + std::to_string(idx);
  ImGui::SameLine(300.f);

  if (ImGui::TreeNode(desc.c_str())) {
    if (ImGui::InputText("name", &t.name)) {
    }

    render_func();

    if (ImGui::Button("Delete")) {
      frame_state::deleted.push_back(idx);
    }
    ImGui::TreePop();
  }
}

void render_parent_gui(ecs::EntityType idx) {
  if (ImGui::Button("Add selected points")) {
    systems::add_sel_points_to_parent(idx);
  }
}

void render_gl_object_gui(gl_object &g,
                          const std::vector<gl_object::draw_mode> &dvals) {
  const std::vector<std::string> mode_names{"points", "lines", "line_strip",
                                            "triangles", "patches"};
  int dmode = to_underlying(g.dmode);
  std::vector<std::string> combovalues;
  for (const auto d : dvals) {
    combovalues.push_back(mode_names[to_underlying(d)]);
  }

  if (ImGui::Combo("draw mode", &dmode, vector_getter,
                   static_cast<void *>(&combovalues), combovalues.size())) {
    g.dmode = dvals[dmode];
  }

  if (ImGui::ColorEdit4("Primary Color", glm::value_ptr(g.primary))) {
  }

  if (ImGui::ColorEdit4("Selection Color", glm::value_ptr(g.selected))) {
    g.color = g.selected;
  }

  float tess[2]{g.tesselation_inner[0], g.tesselation_outer[0]};

  if (ImGui::SliderFloat2("Tesselation Levels", tess, 1.0f, 100.f)) {
    g.tesselation_inner[0] = g.tesselation_inner[1] = tess[0];
    g.tesselation_outer[0] = g.tesselation_outer[1] = g.tesselation_outer[2] =
        g.tesselation_outer[3] = tess[1];
  }
}

void render_relationship_gui(ecs::EntityType idx, relationship &r) {
  static auto &reg = ecs::registry::get_registry();
  ImGui::Text("Life Counter %d", r.indestructible_counter);
  if (r.parents.size()) {
    if (ImGui::TreeNode("Parents")) {
      for (const auto p : r.parents) {
        ImGui::Text("%s##%ld", reg.get_component<tag_figure>(p).name.c_str(),
                    p);
      }
      ImGui::TreePop();
    }
  }
  if (r.children.size()) {
    if (ImGui::TreeNode("Children")) {
      std::optional<ecs::EntityType> del;
      for (const auto c : r.children) {
        ImGui::Text("%s", reg.get_component<tag_figure>(c).name.c_str());
        ImGui::SameLine(200.f);
        if (ImGui::Button(
                (std::string("Detach##") + std::to_string(c)).c_str())) {
          del = c;
        }
      }
      if (del.has_value()) {
        r.children.erase(
            std::find(begin(r.children), end(r.children), del.value()));
        auto &crel = reg.get_component<relationship>(del.value());
        crel.parents.erase(
            std::find(begin(crel.parents), end(crel.parents), idx));
        frame_state::changed.push_back(idx);
      }
      ImGui::TreePop();
    }
  }

  if (r.virtual_children.size()) {
    bool bern = r.virtual_children.size() > 0 &&
                reg.has_component<tag_visible>(r.virtual_children[0]);
    if (ImGui::Checkbox("Virtual points", &bern)) {
      if (bern) {
        for (const auto vc : r.virtual_children) {
          reg.add_component<tag_visible>(vc, {});
        }
      } else {
        for (const auto vc : r.virtual_children) {
          reg.remove_component<tag_visible>(vc);
        }
      }
    }
  }
}

void render_bezier_component_gui(bezierc &bez) {
  static ecs::registry &reg = ecs::registry::get_registry();

  bool visible_bezier = reg.has_component<tag_visible>(bez.bezier_polygon);
  if (ImGui::Checkbox("Bezier polygon", &visible_bezier)) {
    if (visible_bezier) {
      reg.add_component<tag_visible>(bez.bezier_polygon, {});
    } else {
      reg.remove_component<tag_visible>(bez.bezier_polygon);
    }
  }
}

void render_bspline_component_gui(bspline &bsp) {
  static auto &reg = ecs::registry::get_registry();
  bool visible_bezier = reg.has_component<tag_visible>(bsp.bezier_polygon);
  bool visible_deboor = reg.has_component<tag_visible>(bsp.deboor_polygon);

  if (ImGui::Checkbox("Bezier polygon", &visible_bezier)) {
    if (visible_bezier) {
      reg.add_component<tag_visible>(bsp.bezier_polygon, {});
    } else {
      reg.remove_component<tag_visible>(bsp.bezier_polygon);
    }
  }

  if (ImGui::Checkbox("de Boor polygon", &visible_deboor)) {
    if (visible_deboor) {
      reg.add_component<tag_visible>(bsp.deboor_polygon, {});
    } else {
      reg.remove_component<tag_visible>(bsp.deboor_polygon);
    }
  }
}

void render_gregory_component_gui(tag_gregory &tg) {
  static auto &reg = ecs::registry::get_registry();
  bool visible_derivatives = reg.has_component<tag_visible>(tg.derivatives);

  if (ImGui::Checkbox("Derivative lines", &visible_derivatives)) {
    if (visible_derivatives) {
      reg.add_component<tag_visible>(tg.derivatives, {});
    } else {
      reg.remove_component<tag_visible>(tg.derivatives);
    }
  }
}

void render_bsp_component_gui(ecs::EntityType idx, bezier_surface_params &bsp) {
  std::array<int, 2> tmpi{static_cast<int>(bsp.u), static_cast<int>(bsp.v)};
  std::array<float, 2> tmpf{bsp.width, bsp.height};
  bool changed{false};
  if (ImGui::SliderInt2("Num Patches U / V", tmpi.data(), 1, 100)) {
    changed = true;
  }

  if (ImGui::SliderFloat2("Patch Size", tmpf.data(), 1.f, 100.f)) {
    changed = true;
  }

  if (ImGui::Checkbox("Cyllinder", &bsp.cyllinder)) {
    changed = true;
  }

  if (changed) {
    bsp.u = tmpi[0];
    bsp.v = tmpi[1];
    bsp.width = tmpf[0];
    bsp.height = tmpf[1];

    if (bsp.cyllinder) {
      bsp.u = std::max(2u, bsp.u);
    }

    frame_state::changed.push_back(idx);
  }

  if (ImGui::Button("Build")) {
    constructors::add_bezier_surface(idx);
  }
}

void render_bsp_component_gui(ecs::EntityType idx,
                              bspline_surface_params &bsp) {
  std::array<int, 2> tmpi{static_cast<int>(bsp.u), static_cast<int>(bsp.v)};
  std::array<float, 2> tmpf{bsp.width, bsp.height};
  bool changed{false};
  if (ImGui::SliderInt2("Num Patches U / V", tmpi.data(), 1, 100)) {
    changed = true;
  }

  if (ImGui::SliderFloat2("Patch Size", tmpf.data(), 1.f, 100.f)) {
    changed = true;
  }

  if (ImGui::Checkbox("Cyllinder", &bsp.cyllinder)) {
    changed = true;
  }

  if (changed) {
    bsp.u = tmpi[0];
    bsp.v = tmpi[1];
    bsp.width = tmpf[0];
    bsp.height = tmpf[1];

    if (bsp.cyllinder) {
      bsp.u = std::max(3u, bsp.u);
    }

    frame_state::changed.push_back(idx);
  }

  if (ImGui::Button("Build")) {
    constructors::add_bspline_surface(idx);
  }
}

void render_transformation_gui(ecs::EntityType idx, transformation &t) {
  if (ImGui::SliderFloat3("position", glm::value_ptr(t.translation), -100.f,
                          100.f)) {
    frame_state::changed.push_back(idx);
  }
}

void render_point_gui(ecs::EntityType idx, transformation &t, tag_figure &fc) {
  static auto &reg = ecs::registry::get_registry();

  render_figure_gui(idx, fc, [&]() {
    auto &g = reg.get_component<gl_object>(idx);
    std::vector<std::string> cvals{"points"};
    render_gl_object_gui(g, {gl_object::draw_mode::points});

    if (reg.has_component<relationship>(idx)) {
      auto &r = reg.get_component<relationship>(idx);
      render_relationship_gui(idx, r);
    }

    auto &t = reg.get_component<transformation>(idx);
    render_transformation_gui(idx, t);
  });
}

void render_bspline_gui(tag_figure &fc, gl_object &g, relationship &rel,
                        ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();

  render_figure_gui(idx, fc, [&]() {
    auto &g = reg.get_component<gl_object>(idx);
    auto &bsp = reg.get_component<bspline>(idx);

    render_gl_object_gui(
        g, {gl_object::draw_mode::points, gl_object::draw_mode::patches});

    render_bspline_component_gui(bsp);
    render_relationship_gui(idx, rel);
    render_parent_gui(idx);
  });
}

void render_gregory_gui(tag_figure &fc, gl_object &g, relationship &rel,
                        ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();

  render_figure_gui(idx, fc, [&]() {
    auto &g = reg.get_component<gl_object>(idx);
    auto &tg = reg.get_component<tag_gregory>(idx);

    render_gl_object_gui(
        g, {gl_object::draw_mode::points, gl_object::draw_mode::patches});

    render_gregory_component_gui(tg);
    render_relationship_gui(idx, rel);
    render_parent_gui(idx);
  });
}

void render_bezier_surface_gui(tag_figure &fc, bezier_surface_params &bsp,
                               gl_object &g, ecs::EntityType idx) {
  render_figure_gui(idx, fc, [=]() {
    static float v[2]{0.0f, 0.0f};
    auto &reg = ecs::registry::get_registry();
    auto &rel = reg.get_component<relationship>(idx);
    auto &g = reg.get_component<gl_object>(idx);

    render_gl_object_gui(g, {gl_object::draw_mode::patches});
    render_parent_gui(idx);
    render_relationship_gui(idx, rel);
    bool visible_bezier = reg.has_component<tag_visible>(bsp.bezier_polygon);
    if (ImGui::Checkbox("Bezier polygon", &visible_bezier)) {
      if (visible_bezier) {
        reg.add_component<tag_visible>(bsp.bezier_polygon, {});
      } else {
        reg.remove_component<tag_visible>(bsp.bezier_polygon);
      }
    }

    if (ImGui::SliderFloat2("Sampler", v, 0.0f, 1.0f)) {
      auto samp = get_sampler(idx);
      auto pos = samp.sample(v[0], v[1]);
      auto &ctr = reg.get_component<transformation>(
          reg.get_map<cursor_params>().begin()->first);
      ctr.translation = pos;
    }
  });
}

void render_bspline_surface_gui(tag_figure &fc, bspline_surface_params &bsp,
                                gl_object &g, ecs::EntityType idx) {
  render_figure_gui(idx, fc, [=]() {
    auto &reg = ecs::registry::get_registry();
    auto &rel = reg.get_component<relationship>(idx);
    auto &g = reg.get_component<gl_object>(idx);

    render_gl_object_gui(g, {gl_object::draw_mode::patches});
    render_parent_gui(idx);
    render_relationship_gui(idx, rel);
    bool visible_deboor = reg.has_component<tag_visible>(bsp.deboor_polygon);
    if (ImGui::Checkbox("de Boor polygon", &visible_deboor)) {
      if (visible_deboor) {
        reg.add_component<tag_visible>(bsp.deboor_polygon, {});
      } else {
        reg.remove_component<tag_visible>(bsp.deboor_polygon);
      }
    }
  });
}

void render_bezier_surface_builder_gui(tag_figure &fc,
                                       bezier_surface_params &bsp, gl_object &g,
                                       ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();

  render_figure_gui(idx, fc, [&]() {
    auto &g = reg.get_component<gl_object>(idx);

    render_bsp_component_gui(idx, bsp);
    render_gl_object_gui(
        g, {gl_object::draw_mode::points, gl_object::draw_mode::lines});
    render_parent_gui(idx);
  });
}

void render_bspline_surface_builder_gui(tag_figure &fc,
                                        bspline_surface_params &bsp,
                                        gl_object &g, ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();

  render_figure_gui(idx, fc, [&]() {
    auto &g = reg.get_component<gl_object>(idx);

    render_bsp_component_gui(idx, bsp);
    render_gl_object_gui(
        g, {gl_object::draw_mode::points, gl_object::draw_mode::lines});
    render_parent_gui(idx);
  });
}

void render_icurve_gui(tag_figure &fc, gl_object &g, relationship &rel,
                       ecs::EntityType idx) {

  auto &reg = ecs::registry::get_registry();

  render_figure_gui(idx, fc, [&]() {
    auto &g = reg.get_component<gl_object>(idx);

    render_gl_object_gui(
        g, {gl_object::draw_mode::points, gl_object::draw_mode::patches});

    render_relationship_gui(idx, rel);
    render_parent_gui(idx);
  });
}
void render_bezier_gui(tag_figure &fc, gl_object &g, relationship &rel,
                       ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();

  render_figure_gui(idx, fc, [&]() {
    auto &g = reg.get_component<gl_object>(idx);
    auto &bez = reg.get_component<bezierc>(idx);

    render_gl_object_gui(
        g, {gl_object::draw_mode::points, gl_object::draw_mode::patches});

    render_bezier_component_gui(bez);
    render_relationship_gui(idx, rel);
    render_parent_gui(idx);
  });
}

void render_torus_gui(ecs::EntityType idx, torus_params &tp, parametric &p,
                      gl_object &g, transformation &t, tag_figure &fc) {
  using gldm = gl_object::draw_mode;

  static int dmode = 1;
  static std::vector<std::string> combovalues{"points", "lines"};

  auto &reg = ecs::registry::get_registry();

  ImGui::Text("%s", fc.name.c_str());
  std::string desc = ("Show more##") + std::to_string(idx);
  ImGui::SameLine(200.f);
  if (ImGui::TreeNode(desc.c_str())) {
    if (ImGui::Combo("draw mode", &dmode, vector_getter,
                     static_cast<void *>(&combovalues), combovalues.size())) {
      if (dmode == 0) {
        g.dmode = gldm::points;
      } else if (dmode == 1) {
        g.dmode = gldm::line_strip;
      }
    }

    if (ImGui::SliderFloat2("radii", tp.radii, 1.0f, 100.f)) {
      if (tp.radii[1] <= tp.radii[0]) {
        tp.radii[0] = tp.radii[0] > 99.f ? 99.f : tp.radii[0];
        tp.radii[1] = tp.radii[0] + 1;
      }
      g.points.clear();
      g.indices.clear();
      systems::generate_torus_points(tp, p, g.points);
      systems::generate_torus_lines(p, g.points, g.indices);
      systems::reset_gl_objects(g);
    }

    int tmp_samp[2];
    tmp_samp[0] = static_cast<int>(p.samples[0]);
    tmp_samp[1] = static_cast<int>(p.samples[1]);

    if (ImGui::SliderInt2("samples", tmp_samp, 3u, 100u)) {
      p.samples[0] = tmp_samp[0];
      p.samples[1] = tmp_samp[1];
      g.points.clear();
      g.indices.clear();
      systems::generate_torus_points(tp, p, g.points);
      systems::generate_torus_lines(p, g.points, g.indices);
      systems::reset_gl_objects(g);
    }

    auto &name = fc.name;

    if (ImGui::InputText("name", &name)) {
    }

    auto &g = reg.get_component<gl_object>(idx);

    if (ImGui::ColorEdit4("Primary Color", glm::value_ptr(g.primary))) {
    }

    if (ImGui::ColorEdit4("Selection Color", glm::value_ptr(g.selected))) {
    }

    if (ImGui::SliderFloat3("position", glm::value_ptr(t.translation), -100.f,
                            100.f)) {
    }

    if (ImGui::SliderFloat3("rotation", glm::value_ptr(t.rotation), -180.f,
                            180.f)) {
    }

    if (ImGui::SliderFloat3("scale", glm::value_ptr(t.scale), -10.f, 10.f)) {
    }

    if (ImGui::Button("Delete")) {
      frame_state::deleted.push_back(idx);
    }

    ImGui::TreePop();
  }
}

void render_selection_rect() {
  auto &s = input_state::get_input_state();
  auto &reg = ecs::registry::get_registry();
  if (s.mouse_selecting) {
    const auto sel_rect = reg.get_map<tags_selection_rect>().begin()->first;
    auto &t = reg.get_component<transformation>(sel_rect);
    auto &gl = reg.get_component<gl_object>(sel_rect);
    glUseProgram(gl.program);
    update::refresh_identity();
    systems::set_model_uniform(t);
    glVertexAttrib4f(1, gl.color.r, gl.color.g, gl.color.b, gl.color.a);
    systems::render_gl(gl);
    update::refresh_impl(frame_state::col_mat, frame_state::view,
                         frame_state::proj);
  }
}

void end_frame() {
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // update and render additional platform windows
  // (platform functions may change the current opengl context so we
  // save/restore it to make it easier to paste this code elsewhere. For
  // this specific demo appp we could also call
  // glfwMakeCOntextCurrent(window) directly)
  ImGuiIO &io = ImGui::GetIO();
  if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    GLFWwindow *backup_current_context = glfwGetCurrentContext();
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
    glfwMakeContextCurrent(backup_current_context);
  }
}

void render_selected_edit_gui() {
  auto &reg = ecs::registry::get_registry();

  ImGui::Begin("Selected figures:");
  for (auto &[idx, fc] : reg.get_map<selected>()) {
    if (reg.has_component<torus_params>(idx)) {
      auto &tp = reg.get_component<torus_params>(idx);
      auto &t = reg.get_component<transformation>(idx);
      auto &g = reg.get_component<gl_object>(idx);
      auto &p = reg.get_component<parametric>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_torus_gui(idx, tp, p, g, t, fc);
    } else if (reg.has_component<bezierc>(idx)) {
      auto &g = reg.get_component<gl_object>(idx);
      auto &rel = reg.get_component<relationship>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_bezier_gui(fc, g, rel, idx);
    } else if (reg.has_component<bspline>(idx)) {
      auto &g = reg.get_component<gl_object>(idx);
      auto &rel = reg.get_component<relationship>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_bspline_gui(fc, g, rel, idx);
    } else if (reg.has_component<tag_point>(idx) &&
               !reg.has_component<tag_virtual>(idx)) {
      auto &t = reg.get_component<transformation>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_point_gui(idx, t, fc);
    } else if (reg.has_component<icurve>(idx)) {
      auto &g = reg.get_component<gl_object>(idx);
      auto &rel = reg.get_component<relationship>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_icurve_gui(fc, g, rel, idx);
    } else if (reg.has_component<tag_bezier_surface_builder>(idx)) {
      auto &bsp = reg.get_component<bezier_surface_params>(idx);
      auto &g = reg.get_component<gl_object>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_bezier_surface_builder_gui(fc, bsp, g, idx);
    } else if (reg.has_component<bezier_surface_params>(idx)) {
      auto &bsp = reg.get_component<bezier_surface_params>(idx);
      auto &g = reg.get_component<gl_object>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_bezier_surface_gui(fc, bsp, g, idx);
    } else if (reg.has_component<tag_bspline_surface_builder>(idx)) {
      auto &bsp = reg.get_component<bspline_surface_params>(idx);
      auto &g = reg.get_component<gl_object>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_bspline_surface_builder_gui(fc, bsp, g, idx);
    } else if (reg.has_component<bspline_surface_params>(idx)) {
      auto &bsp = reg.get_component<bspline_surface_params>(idx);
      auto &g = reg.get_component<gl_object>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_bspline_surface_gui(fc, bsp, g, idx);
    } else if (reg.has_component<tag_gregory>(idx)) {
      auto &g = reg.get_component<gl_object>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      auto &rel = reg.get_component<relationship>(idx);
      render_gregory_gui(fc, g, rel, idx);
    }
  }
  ImGui::End();
}

void render_figure_select_gui() {
  auto &reg = ecs::registry::get_registry();

  bool sel{false};
  ImGui::Begin("Select figures:");
  for (auto &[idx, fc] : reg.get_map<tag_figure>()) {
    sel = reg.has_component<selected>(idx);
    std::string tree_id = fc.name + ("##") + std::to_string(idx);
    if (ImGui::Selectable(tree_id.c_str(), sel)) {
      if (!ImGui::GetIO().KeyCtrl) { // Clear selection when CTRL is not held
        reg.remove_all<selected>();
      }
      sel = !sel;
      if (sel) {
        if ((reg.get_map<selected>().size() &&
             reg.has_component<tag_virtual>(
                 reg.get_map<selected>().begin()->first))) {
          reg.remove_all<selected>();
        }
        reg.add_component<selected>(idx, {});
      } else {
        reg.remove_component<selected>(idx);
      }
    }
  }

  ImGui::End();
  ImGui::Begin("Group Actions");
  if (ImGui::Button("Delete Selected")) {
    for (auto &[idx, s] : reg.get_map<selected>()) {
      frame_state::deleted.push_back(idx);
    }
  }
  auto &s = reg.get_map<selected>();
  if (s.size() == 2 && reg.has_component<tag_point>(s.begin()->first) &&
      reg.has_component<tag_point>((++s.begin())->first)) {
    if (ImGui::Button("Collapse")) {
      auto id1 = s.begin()->first;
      auto id2 = (++s.begin())->first;

      auto &t1 = reg.get_component<transformation>(id1);
      auto &t2 = reg.get_component<transformation>(id2);
      // collapse two points into each other

      if (!reg.has_component<relationship>(id2)) {
        t1.translation = (t1.translation + t2.translation) * 0.5f;
        frame_state::deleted.push_back(id2);
        frame_state::changed.push_back(id1);
      }

      else if (!reg.has_component<relationship>(id1)) {
        t2.translation = (t1.translation + t2.translation) * 0.5f;
        frame_state::deleted.push_back(id1);
        frame_state::changed.push_back(id2);
      } else {
        // they both have relationships
        // transfer all relationships to t1
        auto &r1 = reg.get_component<relationship>(id1);
        auto &r2 = reg.get_component<relationship>(id2);

        for (auto p : r2.parents) {
          r1.parents.push_back(p);
          auto &prel = reg.get_component<relationship>(p);
          std::replace(prel.children.begin(), prel.children.end(), id2, id1);
        }

        r1.indestructible_counter += r2.indestructible_counter;
        r2.indestructible_counter = 0;
        r2.parents.clear();
        t1.translation = (t1.translation + t2.translation) * 0.5f;
        frame_state::deleted.push_back(id2);
        frame_state::changed.push_back(id1);
      }
    }
  }
  ImGui::End();
}

void render_cursor_gui() {
  static int dmode = 1;
  static std::vector<std::string> combovalues{"Torus",
                                              "Point",
                                              "Bezier Curve C0",
                                              "B-Spline",
                                              "Interpolation Spline",
                                              "Bezier Surface",
                                              "B-Spline Surface",
                                              "Gregory Patch",
                                              "Intersection"};

  auto &reg = ecs::registry::get_registry();

  for (auto &[idx, p] : reg.get_map<cursor_params>()) {
    auto &t = reg.get_component<transformation>(idx);
    std::string title = "cursor";
    ImGui::Begin(title.c_str());

    if (ImGui::SliderFloat3("position", glm::value_ptr(t.translation), -100.f,
                            100.f)) {
    }

    if (ImGui::Combo("current shape", &dmode, vector_getter,
                     static_cast<void *>(&combovalues), combovalues.size())) {
      if (dmode == 0) {
        p.current_shape = cursor_params::cursor_shape::torus;
      } else if (dmode == 1) {
        p.current_shape = cursor_params::cursor_shape::point;
      } else if (dmode == 2) {
        p.current_shape = cursor_params::cursor_shape::bezierc;
      } else if (dmode == 3) {
        p.current_shape = cursor_params::cursor_shape::bspline;
      } else if (dmode == 4) {
        p.current_shape = cursor_params::cursor_shape::icurve;
      } else if (dmode == 5) {
        p.current_shape = cursor_params::cursor_shape::bsurface;
      } else if (dmode == 6) {
        p.current_shape = cursor_params::cursor_shape::bspsurface;
      } else if (dmode == 7) {
        p.current_shape = cursor_params::cursor_shape::gregory;
      } else if (dmode == 8) {
        p.current_shape = cursor_params::cursor_shape::intersection;
      }
    }

    ImGui::End();
  }
}

static void ShowExampleMenuFile() {
  ecs::registry &reg = ecs::registry::get_registry();

  if (ImGui::MenuItem("Reset")) {
    reg.reset();
  }

  if (ImGui::MenuItem("Open", "Ctrl+O")) {
    ImGuiFileDialog::Instance()->OpenDialog("OpenModelChoice",
                                            "Choose Model File", ".json,", ".");
  }

  if (ImGui::MenuItem("Save", "Ctrl+S")) {
    ImGuiFileDialog::Instance()->OpenDialog(
        "SaveModelChoice", "Choose Model Filename", ".json,", ".");
  }

  ImGui::Separator();
  if (ImGui::BeginMenu("Options")) {
    static bool enabled = true;
    ImGui::MenuItem("Enabled", "", &enabled);
    ImGui::BeginChild("child", ImVec2(0, 60), true);
    for (int i = 0; i < 10; i++)
      ImGui::Text("Scrolling Text %d", i);
    ImGui::EndChild();
    static float f = 0.5f;
    static int n = 0;
    ImGui::SliderFloat("Value", &f, 0.0f, 1.0f);
    ImGui::InputFloat("Input", &f, 0.1f);
    ImGui::Combo("Combo", &n, "Yes\0No\0Maybe\0\0");
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Colors")) {
    float sz = ImGui::GetTextLineHeight();
    for (int i = 0; i < ImGuiCol_COUNT; i++) {
      const char *name = ImGui::GetStyleColorName((ImGuiCol)i);
      ImVec2 p = ImGui::GetCursorScreenPos();
      ImGui::GetWindowDrawList()->AddRectFilled(
          p, ImVec2(p.x + sz, p.y + sz), ImGui::GetColorU32((ImGuiCol)i));
      ImGui::Dummy(ImVec2(sz, sz));
      ImGui::SameLine();
      ImGui::MenuItem(name);
    }
    ImGui::EndMenu();
  }

  // Here we demonstrate appending again to the "Options" menu (which we
  // already created above) Of course in this demo it is a little bit silly
  // that this function calls BeginMenu("Options") twice. In a real code-base
  // using it would make senses to use this feature from very different code
  // locations.
  if (ImGui::BeginMenu("Options")) // <-- Append!
  {
    static bool b = true;
    ImGui::Checkbox("SomeOption", &b);
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Disabled", false)) // Disabled
  {
    IM_ASSERT(0);
  }
  if (ImGui::MenuItem("Checked", NULL, true)) {
  }
  if (ImGui::MenuItem("Quit", "Alt+F4")) {
  }
}

void render_popups() {
  // Always center this window when appearing
  ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

  if (ImGui::BeginPopupModal("File Corrupted", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("The file you have pointed to is corrupted or wrongly "
                "formatted!\n\n");
    ImGui::Separator();

    if (ImGui::Button("OK", ImVec2(120, 0))) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopupModal("Hole Not Found", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("These patches don't have a proper hole!\n\n");
    ImGui::Separator();

    if (ImGui::Button("OK", ImVec2(120, 0))) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void render_main_menu() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      ShowExampleMenuFile();
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Edit")) {
      if (ImGui::MenuItem("Undo", "CTRL+Z")) {
      }
      if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {
      } // Disabled item
      ImGui::Separator();
      if (ImGui::MenuItem("Cut", "CTRL+X")) {
      }
      if (ImGui::MenuItem("Copy", "CTRL+C")) {
      }
      if (ImGui::MenuItem("Paste", "CTRL+V")) {
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }

  if (ImGuiFileDialog::Instance()->Display("OpenModelChoice")) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
      std::string filePath = ImGuiFileDialog::Instance()->GetCurrentPath();
      // action
      MG1::SceneSerializer serializer;
      try {
        auto &scene = serializer.LoadScene(filePathName);

        auto &reg = ecs::registry::get_registry();
        reg.load_from_scene(scene);
      } catch (...) {
        ImGui::OpenPopup("File Corrupted");
      }
    }
    ImGuiFileDialog::Instance()->Close();
  }

  if (ImGuiFileDialog::Instance()->Display("SaveModelChoice")) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      auto &reg = ecs::registry::get_registry();
      std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
      // action
      MG1::SceneSerializer serializer;
      reg.get_scene(MG1::Scene::Get());
      serializer.SaveScene(filePathName);
    }
    ImGuiFileDialog::Instance()->Close();
  }
}
} // namespace gui
