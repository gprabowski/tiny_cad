#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <frame_render.h>
#include <frame_update.h>
#include <init.h>
#include <log.h>

void main_loop() {
  static uint64_t begin_time = glfwGetTimerValue();

  auto w = glfwGetCurrentContext();

  while (!glfwWindowShouldClose(w)) {
    render::begin_frame(begin_time);
    render::render_viewport();
    render::render_window_gui();
    update::per_frame_update();
    render::end_frame(w, begin_time);
  }
}

int main() {
  log::init();
  auto glfw_win = init::init_all("tinyCAD");

  main_loop();

  init::cleanup();
}
