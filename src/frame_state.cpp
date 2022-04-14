#include <frame_state.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

glm::mat4 frame_state::proj;
glm::mat4 frame_state::view;

int frame_state::window_w;
int frame_state::window_h;

ImVec2 frame_state::content_area;
ImVec2 frame_state::content_pos;

GLuint frame_state::common_ubo;
GLuint frame_state::common_idx;
int frame_state::common_block_loc;

GLuint frame_state::default_program;

uint64_t frame_state::freq = glfwGetTimerFrequency();
double frame_state::last_cpu_frame;

std::vector<ecs::EntityType> frame_state::changed;
std::set<ecs::EntityType> frame_state::changed_parents;
std::vector<ecs::EntityType> frame_state::deleted;
