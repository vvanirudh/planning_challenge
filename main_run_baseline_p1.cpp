#include "planner_p1.hpp"
#include "terrain_p1.hpp"
int main() {
    World w = load_world();
    std::printf("features %dx%dx2 | demos %zu | start (%.0f,%.0f) goal (%.0f,%.0f)\n",
                w.H, w.W, w.demos.size(), w.start.r, w.start.c, w.goal.r, w.goal.c);
    Grid uniform(w.H, std::vector<double>(w.W, 1.0));
    Path path = astar_plan(w, uniform);
    evaluate(w, path);
    return 0;
}
