#include "component_manager.h"
#include "imgui.h"
#include "imgui_internal.h"
#include <gl_object.h>
#include <gui.h>
#include <string>
#include <systems.h>
#include <torus.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace gui {
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

void render_gui() {
  static bool show_demo = true;
  static bool show_another_window = false;
  static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

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

  if (show_demo) {
    ImGui::ShowDemoWindow(&show_demo);
  }
  // show a simple window that we create ourselves. we use a begin/end pair to
  // create a named window

  if (show_another_window) {
    ImGui::Begin("Another Window", &show_another_window);
    ImGui::Text("Hello from another window!");
    if (ImGui::Button("Close me"))
      show_another_window = false;
    ImGui::End();
  }
}

static auto vector_getter = [](void *vec, int idx, const char **out_text) {
  auto &vector = *static_cast<std::vector<std::string> *>(vec);
  if (idx < 0 || idx >= static_cast<int>(vector.size())) {
    return false;
  }
  *out_text = vector.at(idx).c_str();
  return true;
};

void render_point_gui(ecs::component_manager &cm, ecs::EntityType idx,
                      transformation &t, tag_figure &fc) {
  if (ImGui::TreeNode(fc.name.c_str())) {
    if (ImGui::SliderFloat3("position", glm::value_ptr(t.translation), -100.f,
                            100.f)) {
    }

    if (ImGui::Button("Delete")) {
      cm.delete_entity(idx);
    }

    ImGui::TreePop();
  }
}

void render_torus_gui(ecs::component_manager &cm, ecs::EntityType idx,
                      torus_params &tp, parametric &p, gl_object &g,
                      transformation &t, tag_figure &fc) {
  using gldm = gl_object::draw_mode;

  static int tmp = 0;
  static int dmode = 1;
  static std::vector<std::string> combovalues{"points", "triangles"};

  if (ImGui::TreeNode(fc.name.c_str())) {

    if (ImGui::Combo("draw mode", &dmode, vector_getter,
                     static_cast<void *>(&combovalues), combovalues.size())) {
      if (dmode == 0) {
        g.dmode = gldm::points;
      } else if (dmode == 1) {
        g.dmode = gldm::triangles;
      }
    }

    if (ImGui::SliderFloat2("radii", tp.radii, 1.0f, 100.f)) {
      if (tp.radii[1] <= tp.radii[0]) {
        tp.radii[0] = tp.radii[0] > 99.f ? 99.f : tp.radii[0];
        tp.radii[1] = tp.radii[0] + 1;
      }
      g.points.clear();
      g.indices.clear();
      systems::generate_points(tp, p, g.points);
      systems::generate_triangles(p, g.points, g.indices);
      systems::reset_gl_objects(g);
    }

    if (ImGui::SliderInt2("samples", p.samples, 3u, 100u)) {
      g.points.clear();
      g.indices.clear();
      systems::generate_points(tp, p, g.points);
      systems::generate_triangles(p, g.points, g.indices);
      systems::reset_gl_objects(g);
    }

    if (ImGui::SliderFloat3("position", glm::value_ptr(t.translation), -100.f,
                            100.f)) {
      // cool
    }

    if (ImGui::SliderFloat3("rotation", glm::value_ptr(t.rotation), -180.f,
                            180.f)) {
      t.qrotation = glm::quat(glm::radians(t.rotation));
    }

    if (ImGui::Button("Delete")) {
      cm.delete_entity(idx);
    }
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::TreePop();
  }
}

void show_gui() {
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

void render_selected_edit_gui(ecs::component_manager &cm) {
  ImGui::Begin("Selected figures:");
  for (auto &[idx, fc] : cm.selected_component) {
    if (cm.has_component<torus_params>(idx)) {
      auto &tp = cm.get_component<torus_params>(idx);
      auto &t = cm.get_component<transformation>(idx);
      auto &g = cm.get_component<gl_object>(idx);
      auto &p = cm.get_component<parametric>(idx);
      auto &fc = cm.get_component<tag_figure>(idx);
      render_torus_gui(cm, idx, tp, p, g, t, fc);
    } else if (cm.has_component<tag_point>(idx)) {
      auto &t = cm.get_component<transformation>(idx);
      auto &fc = cm.get_component<tag_figure>(idx);
      render_point_gui(cm, idx, t, fc);
    }
  }
  ImGui::End();
}

void render_figure_edit_gui(ecs::component_manager &cm) {
  ImGui::Begin("All figures:");
  for (auto &[idx, fc] : cm.figure_component) {
    if (cm.has_component<torus_params>(idx)) {
      auto &tp = cm.get_component<torus_params>(idx);
      auto &t = cm.get_component<transformation>(idx);
      auto &g = cm.get_component<gl_object>(idx);
      auto &p = cm.get_component<parametric>(idx);
      render_torus_gui(cm, idx, tp, p, g, t, fc);
    } else if (cm.has_component<tag_point>(idx)) {
      auto &t = cm.get_component<transformation>(idx);
      render_point_gui(cm, idx, t, fc);
    }
  }
  ImGui::End();
}

void render_figure_select_gui(ecs::component_manager &cm) {
  static std::bitset<1024> selection{0x0};
  static std::bitset<1024> component_set{0x0};

  ImGui::Begin("Select figures:");

  for (auto &[idx, fc] : cm.figure_component) {
    if (ImGui::Selectable(fc.name.c_str(), selection[idx])) {
      if (!ImGui::GetIO().KeyCtrl) // Clear selection when CTRL is not held
        selection.reset();
      selection.flip(idx);

      if (selection[idx]) {
        cm.add_component<selected>(idx, {});
      } else {
        cm.remove_component<selected>(idx);
      }
    }
  }

  ImGui::End();
}

void render_cursor_gui(ecs::component_manager &cm) {
  static int dmode = 0;
  static std::vector<std::string> combovalues{"torus", "point"};

  for (auto &[idx, p] : cm.cursor_component) {
    auto &t = cm.get_component<transformation>(idx);
    std::string title = "cursor";
    ImGui::Begin(title.c_str());

    if (ImGui::SliderFloat3("position", glm::value_ptr(t.translation), -100.f,
                            100.f)) {
      // cool
    }

    if (ImGui::Combo("current shape", &dmode, vector_getter,
                     static_cast<void *>(&combovalues), combovalues.size())) {
      if (dmode == 0) {
        p.current_shape = cursor_params::cursor_shape::torus;
      } else if (dmode == 1) {
        p.current_shape = cursor_params::cursor_shape::point;
      }
    }

    ImGui::End();
  }
}
} // namespace gui
