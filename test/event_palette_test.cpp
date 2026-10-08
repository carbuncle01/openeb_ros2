#include "openeb_ros2/event_palette.hpp"
#include <cassert>
int main() {
  using openeb_ros2::red_blue_white;
  using Color = std::array<std::uint8_t,3>;
  assert((red_blue_white(0,0) == Color{255,255,255}));
  assert((red_blue_white(1,0) == Color{0,0,255}));
  assert((red_blue_white(0,1) == Color{255,0,0}));
  assert((red_blue_white(1,1) == Color{0,0,255}));
  assert((red_blue_white(0.5F,0) == Color{128,128,255}));
  assert((red_blue_white(10,-1) == Color{0,0,255}));
}
