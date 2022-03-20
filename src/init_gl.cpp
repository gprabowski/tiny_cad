#include "glad/glad.h"

#include <cstdio>
#include <cstdlib>

#include <init_gl.h>

static void framebuffer_size_callback(GLFWwindow *window, int width,
                                      int height) {
  glViewport(0, 0, width, height);
}

void glfw_window_destroyer(GLFWwindow *w) {
  printf("Destroying GLFW window\n");
  glfwDestroyWindow(w);
}

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "[GLFW ERROR] %d: %s\n", error, description);
}

void teardown() { glfwTerminate(); }

void glfw_die(const char *message) {
  const char *err;
  glfwGetError(&err);
  fprintf(stderr, "%s: %s\n", message, err);
  exit(-1);
}

static void APIENTRY openglCallbackFunction(GLenum source, GLenum type,
                                            GLuint id, GLenum severity,
                                            GLsizei length,
                                            const GLchar *message,
                                            const void *userParam) {
  (void)source;
  (void)type;
  (void)id;
  (void)severity;
  (void)length;
  (void)userParam;
  fprintf(stderr, "%s\n", message);
  if (severity == GL_DEBUG_SEVERITY_HIGH) {
    fprintf(stderr, "Aborting...\n");
    abort();
  }
}

namespace init_gl {
std::shared_ptr<GLFWwindow> init_screen(const char *caption) {
  static constexpr int init_w = 1600;
  static constexpr int init_h = 900;

  glfwSetErrorCallback(glfw_error_callback);
  atexit(teardown);

  if (!glfwInit()) {
    glfw_die("[ERROR] Couldn't initialize GLFW\n");
  }

  printf("[INFO] GLFW initialized\n");

  // Setup GLFW window
  glfwWindowHint(GLFW_DOUBLEBUFFER, 1);
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_STENCIL_BITS, 8);
  glfwWindowHint(GLFW_SAMPLES, 4);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);

#ifndef RELEASE_MODE
  // Debugging
  glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
#endif

  constexpr float high_dpi_scale_factor = 1.0f;

  auto w = std::shared_ptr<GLFWwindow>(
      glfwCreateWindow(init_w, init_h, caption, NULL, NULL),
      glfw_window_destroyer);

  if (!w) {
    glfw_die("[ERROR] Couldn't create a GLFW window");
  }

  glfwMakeContextCurrent(w.get());

  glfwSetFramebufferSizeCallback(w.get(), framebuffer_size_callback);
  // VSync
  glfwSwapInterval(1);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    glfw_die("[ERROR] Couldn't initialize GLAD");
  } else {
    printf("[INFO] GLAD initialized\n");
  }

  printf("OpenGL loaded\n");
  printf("Vendor: %s\n", glGetString(GL_VENDOR));
  printf("Renderer: %s\n", glGetString(GL_RENDERER));
  printf("Version: %s\n", glGetString(GL_VERSION));

#ifndef RELEASE_MODE
  glEnable(GL_DEBUG_OUTPUT);
  glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
  glDebugMessageCallback(openglCallbackFunction, nullptr);
  glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL,
                        true);
#endif

  // glEnable(GL_CULL_FACE);

  int actualWindowWidth, actualWindowHeight;
  glfwGetWindowSize(w.get(), &actualWindowWidth, &actualWindowHeight);
  glViewport(0, 0, actualWindowWidth, actualWindowHeight);
  glClearColor(0.1f, 0.3f, 0.2f, 1.0f);

  return w;
}
} // namespace init_gl
