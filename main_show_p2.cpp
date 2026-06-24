#include "terrain_p2.hpp"
int main() {
    World w = load_world();
    std::printf("field %dx%dx3 | start (%.0f,%.0f) goal (%.0f,%.0f)\n",
                w.H, w.W, w.start.x, w.start.y, w.goal.x, w.goal.y);
    show(w, {}, nullptr, "initial_p2.bin");
    return 0;
}
