#include "learn_p1.hpp"
#include "planner_p1.hpp"
int main() {
    World w = load_world();
    auto cm = cost_map_learned(w);
    Path path = astar_plan(w, cm);
    show(w, {path}, &cm, "run_cost_p1.bin");
    show(w, {path}, nullptr, "run_terrain_p1.bin");
    evaluate(w, path, cm);
    return 0;
}
