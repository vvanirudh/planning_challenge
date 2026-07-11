#include "learn_p1.hpp"
#include "planner_p1.hpp"
int main() {
    World w = load_world();
    Grid cm = cost_map_learned(w);
    Path path = astar_plan(w, cm);
    show(w, {path}, &cm,     "run_cost_p1.bin");    // over the learned cost map
    show(w, {path}, nullptr, "run_true_p1.bin");    // over the true cost map
    evaluate(w, path);
    return 0;
}
