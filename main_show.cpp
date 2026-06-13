#include "terrain.hpp"
int main() {
    World w = load_world();
    std::printf("field %dx%dx3 | demos %zu | start (%.0f,%.0f) goal (%.0f,%.0f)\n",
                w.H, w.W, w.demos.size(), w.start.x, w.start.y, w.goal.x, w.goal.y);
    // No candidate path yet: render the hidden true cost map + the demos (gray).
    show(w, {}, nullptr, "initial.bin");
    return 0;
}
