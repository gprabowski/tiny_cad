#include <frame_state.h>

glm::mat4 frame_state::proj;
glm::mat4 frame_state::view;

int frame_state::window_w;
int frame_state::window_h;

GLuint frame_state::common_ubo;
GLuint frame_state::common_idx;
int frame_state::common_block_loc;
