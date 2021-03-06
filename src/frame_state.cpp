#include <frame_state.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

glm::mat4 frame_state::proj;
glm::mat4 frame_state::view;
glm::mat4 frame_state::col_mat = glm::mat4(1.0f);
glm::mat4 frame_state::col_mat_left = glm::mat4(1.0f);
glm::mat4 frame_state::col_mat_right = glm::mat4(1.0f);

int frame_state::window_w;
int frame_state::window_h;

ImVec2 frame_state::content_area;
ImVec2 frame_state::content_pos;

uint64_t frame_state::freq = glfwGetTimerFrequency();
double frame_state::last_cpu_frame;

std::vector<ecs::EntityType> frame_state::changed;
std::set<ecs::EntityType> frame_state::changed_parents;
std::vector<ecs::EntityType> frame_state::deleted;
