// Pre-provided baseline cost (do not edit). Wrong-but-runnable (H,W) linear
// cost: 1 + 5*slope (ignores mud), rocks at ROCK_PENALTY. Routing the provided
// planner with this drives through the mud band and FAILs the bar.
#pragma once
#include "metric_p1.hpp"
#include "terrain_p1.hpp"

inline std::vector<double> cost_map_baseline(const World& w) {
    std::vector<double> cm(size_t(w.H)*w.W);
    for (int r = 0; r < w.H; ++r) for (int c = 0; c < w.W; ++c) {
        double v = 1.0 + 5.0 * w.fld(r,c,SLOPE);
        if (w.fld(r,c,ROCK) == 1.0) v = ROCK_PENALTY;
        cm[size_t(r)*w.W + c] = v;
    }
    return cm;
}
