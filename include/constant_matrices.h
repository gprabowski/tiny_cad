#define GLM_FORCE_RADIANS
#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

struct constant_matrices {
  static constexpr glm::mat4 anaglyph_mono_left{{1.f, 0.f, 0.f, 0.f},
                                                {0, 1.f, 0.f, 0.f},
                                                {0.f, 0.f, 1.f, 0.f},
                                                {0.f, 0.f, 0.f, 1.f}};

  static constexpr glm::mat4 anaglyph_mono_right{{1.f, 0.f, 0.f, 0.f},
                                                 {0, 1.f, 0.f, 0.f},
                                                 {0.f, 0.f, 1.f, 0.f},
                                                 {0.f, 0.f, 0.f, 1.f}};

  // Dark image
  // No color reproduction
  // Little ghosting

  static constexpr glm::mat4 anaglyph_true_left{{0.299f, 0.0f, 0.0f, 0.0f},
                                                {0.587f, 0.0f, 0.0f, 0.0f},
                                                {0.114f, 0.f, 0.f, 0.f},
                                                {0.f, 0.f, 0.f, 1.f}};

  static constexpr glm::mat4 anaglyph_true_right{{0.f, 0.f, 0.299f, 0.f},
                                                 {0, 0.f, 0.587f, 0.f},
                                                 {0, 0.f, 0.114f, 0.f},
                                                 {0.f, 0.f, 0.f, 1.f}};

  // No color reproduction
  // More ghosting than true anaglyphs

  static constexpr glm::mat4 anaglyph_gray_left{{0.299f, 0.f, 0.f, 0.f},
                                                {0.587, 0.f, 0.f, 0.f},
                                                {0.114f, 0.f, 0.f, 0.f},
                                                {0.f, 0.f, 0.f, 1.f}};

  static constexpr glm::mat4 anaglyph_gray_right{{0.f, 0.299f, 0.299f, 0.f},
                                                 {0.f, 0.587f, 0.587f, 0.f},
                                                 {0.f, 0.114f, 0.114f, 0.f},
                                                 {0.f, 0.f, 0.f, 1.f}};

  // Partial color reproduction
  // Retinal rivalry

  static constexpr glm::mat4 anaglyph_color_left{{1.f, 0.f, 0.f, 0.f},
                                                 {0.f, 0.f, 0.f, 0.f},
                                                 {0.f, 0.f, 0.f, 0.f},
                                                 {0.f, 0.f, 0.f, 1.f}};

  static constexpr glm::mat4 anaglyph_color_right{{0.f, 0.f, 0.f, 0.f},
                                                  {0.f, 1.f, 0.f, 0.f},
                                                  {0.f, 0.f, 1.f, 0.f},
                                                  {0.f, 0.f, 0.f, 1.f}};

  // Partial color reproduction (but not as good as color anaglyphs)
  // Less retinal rivalry than color anaglyphs

  static constexpr glm::mat4 anaglyph_half_color_left{{0.299f, 0.f, 0.f, 0.f},
                                                      {0.587, 0.f, 0.f, 0.f},
                                                      {0.114f, 0.f, 0.f, 0.f},
                                                      {0.f, 0.f, 0.f, 1.f}};

  static constexpr glm::mat4 anaglyph_half_color_right{{0.f, 0.f, 0.f, 0.f},
                                                       {0, 1.f, 0.f, 0.f},
                                                       {0.f, 0.f, 1.f, 0.f},
                                                       {0.f, 0.f, 0.f, 1.f}};

  // Partial color reproduction (but not of red shades)
  // Almost no retinal rivalry

  static constexpr glm::mat4 anaglyph_optimized_left{{0.f, 0.f, 0.f, 0.f},
                                                     {0.7, 0.f, 0.f, 0.f},
                                                     {0.3f, 0.f, 0.f, 0.f},
                                                     {0.f, 0.f, 0.f, 1.f}};

  static constexpr glm::mat4 anaglyph_optimized_right{{0.f, 0.f, 0.f, 0.f},
                                                      {0, 1.f, 0.f, 0.f},
                                                      {0.f, 0.f, 1.f, 0.f},
                                                      {0.f, 0.f, 0.f, 1.f}};
  // custom to be chosen from GUI

  static constexpr glm::mat4 anaglyph_custom_left{{1.f, 0.f, 0.f, 0.f},
                                                  {0, 1.f, 0.f, 0.f},
                                                  {0.f, 0.f, 1.f, 0.f},
                                                  {0.f, 0.f, 0.f, 1.f}};

  static constexpr glm::mat4 anaglyph_custom_right{{1.f, 0.f, 0.f, 0.f},
                                                   {0, 1.f, 0.f, 0.f},
                                                   {0.f, 0.f, 1.f, 0.f},
                                                   {0.f, 0.f, 0.f, 1.f}};
};
