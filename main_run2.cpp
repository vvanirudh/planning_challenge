#include "learn.hpp"
#include "planner.hpp"
int main() {
    World w = load_world();
    auto cm = cost_map_learned(w);
    Path path = plan(w, cm);
    show(w, {path}, &cm, "run2_cost.bin");
    show(w, {path}, nullptr, "run2_terrain.bin");
    evaluate(w, path, cm);
    return 0;
}
