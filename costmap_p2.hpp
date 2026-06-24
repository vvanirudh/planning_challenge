// The GIVEN cost map (do not edit). This IS the objective: minimize the
// path-integral of this (H,W) per-cell cost from start to goal. Cells set to
// ROCK_PENALTY are untraversable (obstacles). Pass it to plan() and evaluate().
#pragma once
#include "metric_p2.hpp"
#include "terrain_p2.hpp"

inline std::vector<double> cost_map(const World& w) {
    std::vector<double> cm(size_t(w.H)*w.W);
    for (int r = 0; r < w.H; ++r) for (int c = 0; c < w.W; ++c) {
        double v = 1.0 + 6.0 * w.fld(r,c,MUD) + 4.0 * w.fld(r,c,SLOPE);
        if (w.fld(r,c,ROCK) == 1.0) v = ROCK_PENALTY;
        cm[size_t(r)*w.W + c] = v;
    }
    return cm;
}
