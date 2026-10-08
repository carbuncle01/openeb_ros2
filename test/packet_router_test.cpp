#include "openeb_ros2/packet_router.hpp"
#include <cassert>
#include <functional>
#include <vector>
struct Packet {
  static int copies;
  int sequence;
  std::vector<unsigned char> bytes;
  Packet(int s): sequence(s), bytes{1,2,3} {}
  Packet(const Packet & other): sequence(other.sequence), bytes(other.bytes) { ++copies; }
};
int Packet::copies = 0;
int main() {
  using Pointer = std::unique_ptr<Packet>;
  for (bool direct : {false, true}) {
    for (bool enabled : {false, true}) {
      int delivered = 0, published = 0;
      std::function<void(Pointer)> sink;
      auto packet = std::make_unique<Packet>(42);
      auto * original = packet.get();
      Packet::copies = 0;
      if (direct) sink = [&](Pointer p) {
        assert(p.get() == original); assert(p->sequence == 42);
        assert(p->bytes == std::vector<unsigned char>({1,2,3})); ++delivered;
      };
      auto publish = [&](Pointer p) {
        assert(p->sequence == 42);
        assert(p->bytes == std::vector<unsigned char>({1,2,3})); ++published;
      };
      openeb_ros2::route_packet(std::move(packet), sink, publish, enabled);
      assert(delivered == int(direct));
      assert(published == int(enabled));
      assert(Packet::copies == int(direct && enabled));
    }
  }
  // Changing the monitor gate must not interrupt direct processing or alter order.
  std::vector<int> input, output;
  std::function<void(Pointer)> sink = [&](Pointer p) {input.push_back(p->sequence);};
  auto publish = [&](Pointer p) {output.push_back(p->sequence);};
  for (int i=0;i<5;++i) openeb_ros2::route_packet(std::make_unique<Packet>(i),sink,publish,i%2==0);
  assert(input == std::vector<int>({0,1,2,3,4}));
  assert(output == std::vector<int>({0,2,4}));
}
