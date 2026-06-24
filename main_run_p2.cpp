#include "costmap_p2.hpp"
#include "planner_p2.hpp"
int main() {
    World w = load_world();
    auto cm = cost_map(w);
    Path path = plan(w, cm);
    show(w, {path}, &cm, "run_p2.bin");
    evaluate(w, path, cm);
    return 0;
}
