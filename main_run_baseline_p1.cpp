#include "baseline_p1.hpp"
#include "planner_p1.hpp"
int main() {
    World w = load_world();
    std::printf("field %dx%dx3 | demos %zu | start (%.0f,%.0f) goal (%.0f,%.0f)\n",
                w.H, w.W, w.demos.size(), w.start.x, w.start.y, w.goal.x, w.goal.y);
    auto cm = cost_map_baseline(w);
    Path path = astar_plan(w, cm);
    show(w, {path}, &cm, "run_baseline_p1.bin");
    evaluate(w, path, cm);
    return 0;
}
