#include "baseline.hpp"
#include "planner.hpp"
int main() {
    World w = load_world();
    std::printf("field %dx%dx3 | demos %zu | start (%.0f,%.0f) goal (%.0f,%.0f)\n",
                w.H, w.W, w.demos.size(), w.start.x, w.start.y, w.goal.x, w.goal.y);
    auto cm = cost_map_baseline(w);
    Path path = plan(w, cm);
    show(w, {path}, &cm, "run1.bin");
    evaluate(w, path, cm);
    return 0;
}
