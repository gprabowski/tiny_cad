#include "registry.h"

#include "imgui.h"
#include "imgui_internal.h"
#include <misc/cpp/imgui_stdlib.h>

#include <algorithm>
#include <optional>
#include <string>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <ImGuizmo.h>

#include <gl_object.h>
#include <gui.h>
#include <systems.h>
#include <torus.h>

namespace gui {

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
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
}

void cleanup_gui() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
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

  ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(), dockspace_flags);
  ImGuizmo::BeginFrame();

  if (show_demo) {
    ImGui::ShowDemoWindow(&show_demo);
  }
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
  ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
  ImGui::End();
}

point_action render_point_gui(ecs::EntityType idx, transformation &t,
                              tag_figure &fc) {
  auto &reg = ecs::registry::get_registry();

  point_action ret{point_action::none};
  std::string tree_id = fc.name + ("##") + std::to_string(idx);

  ImGui::Text("%s", fc.name.c_str());
  std::string desc = ("Show more##") + std::to_string(idx);
  ImGui::SameLine(200.f);
  if (ImGui::TreeNode(desc.c_str())) {
    if (ImGui::SliderFloat3("position", glm::value_ptr(t.translation), -100.f,
                            100.f)) {
      ret = point_action::edit;
    }

    auto &name = reg.get_component<tag_figure>(idx).name;

    if (ImGui::InputText("name", &name)) {
    }

    if (reg.has_component<relationship>(idx)) {
      auto &rel = reg.get_component<relationship>(idx);
      if (ImGui::TreeNode("Parents")) {
        for (const auto p : rel.parents) {
          ImGui::Text("%s##%ld", reg.get_component<tag_figure>(p).name.c_str(),
                      p);
        }
        ImGui::TreePop();
      }
    }

    if (ImGui::Button("Delete")) {
      ret = point_action::del;
    }

    ImGui::TreePop();
  }
  return ret;
}

void render_bspline_gui(tag_figure &fc, gl_object &g, relationship &rel,
                        ecs::EntityType idx,
                        std::vector<ecs::EntityType> &changed) {

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

    auto &name = fc.name;

    if (ImGui::InputText("name", &name)) {
    }

    auto &sec = reg.get_component<secondary_object>(idx);
    if (ImGui::Checkbox("Bounding polygon", &sec.enabled)) {
      if (sec.enabled) {
        reg.add_component<tag_visible>(sec.val, {});
      } else {
        reg.remove_component<tag_visible>(sec.val);
      }
    }
    bool bern = rel.virtual_children.size() > 0 &&
                reg.has_component<tag_visible>(rel.virtual_children[0]);
    if (ImGui::Checkbox("Bezier points", &bern)) {
      if (bern) {
        for (const auto vc : rel.virtual_children) {
          reg.add_component<tag_visible>(vc, {});
        }
      } else {
        for (const auto vc : rel.virtual_children) {
          reg.remove_component<tag_visible>(vc);
        }
      }
    }

    if (ImGui::TreeNode("Children")) {
      std::optional<ecs::EntityType> del;
      for (const auto c : rel.children) {
        ImGui::Text("%s", reg.get_component<tag_figure>(c).name.c_str());
        ImGui::SameLine(200.f);
        if (ImGui::Button(
                (std::string("Detach##") + std::to_string(c)).c_str())) {
          del = c;
        }
      }
      if (del.has_value()) {
        rel.children.erase(
            std::find(begin(rel.children), end(rel.children), del.value()));
        auto &crel = reg.get_component<relationship>(del.value());
        crel.parents.erase(
            std::find(begin(crel.parents), end(crel.parents), idx));
        changed.push_back(idx);
      }
      ImGui::TreePop();
    }

    if (ImGui::Button("Add selected points")) {
      systems::add_sel_points_to_parent(idx);
    }

    if (ImGui::Button("Delete")) {
      reg.delete_entity(idx);
    }
    ImGui::TreePop();
  }
}
void render_bezier_gui(tag_figure &fc, gl_object &g, relationship &rel,
                       ecs::EntityType idx,
                       std::vector<ecs::EntityType> &changed) {
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

    auto &name = fc.name;

    if (ImGui::InputText("name", &name)) {
    }

    auto &sec = reg.get_component<secondary_object>(idx);
    if (ImGui::Checkbox("Bounding polygon", &sec.enabled)) {
      if (sec.enabled) {
        reg.add_component<tag_visible>(idx, {});
      } else {
        reg.remove_component<tag_visible>(idx);
      }
    }

    if (ImGui::TreeNode("Children")) {
      std::optional<ecs::EntityType> del;
      for (const auto c : rel.children) {
        ImGui::Text("%s", reg.get_component<tag_figure>(c).name.c_str());
        ImGui::SameLine(300.f);
        if (ImGui::Button(
                (std::string("Detach##") + std::to_string(c)).c_str())) {
          del = c;
        }
      }
      if (del.has_value()) {
        rel.children.erase(
            std::find(begin(rel.children), end(rel.children), del.value()));
        auto &crel = reg.get_component<relationship>(del.value());
        crel.parents.erase(
            std::find(begin(crel.parents), end(crel.parents), idx));
        changed.push_back(idx);
      }
      ImGui::TreePop();
    }

    if (ImGui::Button("Add selected points")) {
      systems::add_sel_points_to_parent(idx);
    }

    if (ImGui::Button("Delete")) {
      reg.delete_entity(idx);
    }
    ImGui::TreePop();
  }
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

void render_selected_edit_gui(std::vector<ecs::EntityType> &changed,
                              std::vector<ecs::EntityType> &deleted) {
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
    } else if (reg.has_component<tag_bezierc>(idx)) {
      auto &g = reg.get_component<gl_object>(idx);
      auto &rel = reg.get_component<relationship>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_bezier_gui(fc, g, rel, idx, changed);
    } else if (reg.has_component<tag_bspline>(idx)) {
      auto &g = reg.get_component<gl_object>(idx);
      auto &rel = reg.get_component<relationship>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      render_bspline_gui(fc, g, rel, idx, changed);
    } else if (reg.has_component<tag_point>(idx) &&
               !reg.has_component<tag_virtual>(idx)) {
      auto &t = reg.get_component<transformation>(idx);
      auto &fc = reg.get_component<tag_figure>(idx);
      switch (render_point_gui(idx, t, fc)) {
      case point_action::none: {
      } break;
      case point_action::edit: {
        changed.push_back(idx);
      } break;
      case point_action::del: {
        deleted.push_back(idx);
      } break;
      }
    }
  }
  ImGui::End();
}

point_action render_figure_select_gui(std::vector<ecs::EntityType> &deleted) {
  auto &reg = ecs::registry::get_registry();

  point_action ret{point_action::none};
  bool sel{false};
  ImGui::Begin("Select figures:");
  for (auto &[idx, fc] : reg.get_map<tag_figure>()) {
    sel = reg.has_component<selected>(idx);
    std::string tree_id = fc.name + ("##") + std::to_string(idx);
    if (ImGui::Selectable(tree_id.c_str(), sel)) {
      if (!ImGui::GetIO().KeyCtrl) { // Clear selection when CTRL is not held
        reg.clear_component<selected>();
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
      deleted.push_back(idx);
    }
  }
  ImGui::End();
  return ret;
}

void render_cursor_gui() {
  static int dmode = 1;
  static std::vector<std::string> combovalues{"Torus", "Point",
                                              "Bezier Curve C0", "B-Spline"};

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
      }
    }

    ImGui::End();
  }
}
} // namespace gui
