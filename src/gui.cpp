#include <registry.h>

#include "imgui.h"
#include "imgui_internal.h"
#include <misc/cpp/imgui_stdlib.h>

#include <implot/implot.h>

#include <algorithm>
#include <optional>
#include <string>
#include <utility>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <ImGuizmo.h>

#include <frame_state.h>
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

  static std::vector<std::string> wasd_values{"Camera Movement",
                                              "Cursor Movement"};

  static std::vector<std::string> gizmo_values{"Translation", "Rotation",
                                               "Scaling", "Universal"};
  ImGui::Begin("General Settings");

  static input_state::gizmo_mode gmode{input_state::gizmo_mode::universal};
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
  }
}

void render_relationship_gui(ecs::EntityType idx, relationship &r) {
  static auto &reg = ecs::registry::get_registry();
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

    if (ImGui::SliderInt2("samples", p.samples, 3u, 100u)) {
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
      reg.delete_entity(idx);
    }
    ImGui::TreePop();
  }
}

void end_frame() {
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // update and render additional platform windows
  // (platform functions may change the current opengl context so we
  // save/restore it to make it easier to paste this code elsewhere. For this
  // specific demo appp we could also call glfwMakeCOntextCurrent(window)
  // directly)
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
  ImGui::End();
}

void render_cursor_gui() {
  static int dmode = 1;
  static std::vector<std::string> combovalues{
      "Torus", "Point", "Bezier Curve C0", "B-Spline", "Interpolation Spline"};

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
      }
    }

    ImGui::End();
  }
}
} // namespace gui
