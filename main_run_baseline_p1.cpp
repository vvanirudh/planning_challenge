#include "planner_p1.hpp"
#include "terrain_p1.hpp"
int main() {
    World w = load_world();
    std::printf("features %dx%dx3 | demos %zu | start (%.0f,%.0f) goal (%.0f,%.0f)\n",
                w.H, w.W, w.demos.size(), w.start.r, w.start.c, w.goal.r, w.goal.c);
    Path path = astar_plan(w, w.example_cost_map);
    evaluate(w, path, w.example_cost_map);
    return 0;
}
