#ifndef OPENEB_ROS2__PACKET_ROUTER_HPP_
#define OPENEB_ROS2__PACKET_ROUTER_HPP_
#include <memory>
#include <utility>
namespace openeb_ros2 {
// Publication is optional; direct processing always receives the original buffer.
// The only extra packet copy is the opt-in monitor branch.
template<class Packet, class Sink, class Publisher>
void route_packet(std::unique_ptr<Packet> packet, Sink & sink,
  Publisher publish, bool publish_enabled)
{
  if (sink) {
    auto monitor = publish_enabled ? std::make_unique<Packet>(*packet) : nullptr;
    sink(std::move(packet));
    if (monitor) { publish(std::move(monitor)); }
  } else if (publish_enabled) {
    publish(std::move(packet));
  }
}
} // namespace openeb_ros2
#endif
