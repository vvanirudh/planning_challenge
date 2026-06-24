// PRE-BUILT METRIC (do not edit) -- exact port of the Python path_cost / evaluate.
#pragma once
#include "terrain_p2.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>

constexpr double ROCK_PENALTY = 1e6;   // penalty for rock / forbidden / OOB cells
constexpr double STEP = 0.5;           // path-integral resampling spacing (cells)

inline bool resample_steps(const World& w, const Path& path,
                           std::vector<int>& rx, std::vector<int>& ry,
                           std::vector<bool>& in_bounds) {
    std::vector<double> seg(path.size() ? path.size()-1 : 0);
    double L = 0;
    for (size_t k = 1; k < path.size(); ++k) {
        double dx = path[k].x-path[k-1].x, dy = path[k].y-path[k-1].y;
        seg[k-1] = std::sqrt(dx*dx+dy*dy); L += seg[k-1];
    }
    if (L < 1e-9) return false;
    std::vector<double> s(path.size());
    s[0] = 0; for (size_t k = 1; k < path.size(); ++k) s[k] = s[k-1]+seg[k-1];
    int n = std::max(2, int(L/STEP) + 1);
    rx.resize(n); ry.resize(n); in_bounds.assign(n, false);
    for (int j = 0; j < n; ++j) {
        double u = L * j / double(n-1);
        size_t k = 1; while (k < s.size()-1 && s[k] < u) ++k;
        double t = (s[k]-s[k-1]>1e-12) ? (u-s[k-1])/(s[k]-s[k-1]) : 0.0;
        double x = path[k-1].x + (path[k].x-path[k-1].x)*t;
        double y = path[k-1].y + (path[k].y-path[k-1].y)*t;
        int ix = (int)std::lround(x), iy = (int)std::lround(y);
        rx[j] = ix; ry[j] = iy;
        in_bounds[j] = (ix >= 0 && ix <= w.W-1 && iy >= 0 && iy <= w.H-1);
    }
    return true;
}

inline double path_cost(const World& w, const Path& path,
                        const std::vector<double>& cost_map) {
    std::vector<int> rx, ry; std::vector<bool> ib;
    if (!resample_steps(w, path, rx, ry, ib)) return INFINITY;
    double total = 0;
    for (size_t j = 0; j < rx.size(); ++j) {
        if (ib[j]) {
            int cx = std::min(std::max(rx[j],0), w.W-1);
            int cy = std::min(std::max(ry[j],0), w.H-1);
            total += cost_map[size_t(cy)*w.W + cx];
        } else total += ROCK_PENALTY;
    }
    return total * STEP;
}

// Official PASS/FAIL scorer. Checks (1) endpoints near start/goal, (2) feasible
// under cost_map, (3) in bounds, (4) no rock, (5) cost <= near-optimal bar.
inline bool evaluate(const World& w, const Path& path,
                     const std::vector<double>& cost_map) {
    const double HI = ROCK_PENALTY / 2.0;
    double d0x = path.front().x-w.start.x, d0y = path.front().y-w.start.y;
    double d1x = path.back().x -w.goal.x,  d1y = path.back().y -w.goal.y;
    bool start_ok = std::sqrt(d0x*d0x+d0y*d0y) < 5;
    bool goal_ok  = std::sqrt(d1x*d1x+d1y*d1y) < 5;

    std::vector<int> rx, ry; std::vector<bool> ib;
    bool model_feasible=false, rock_hit=false, out_of_bounds=false;
    if (resample_steps(w, path, rx, ry, ib)) {
        out_of_bounds = false; model_feasible = true; rock_hit = false;
        for (size_t j = 0; j < rx.size(); ++j) {
            if (!ib[j]) { out_of_bounds = true; continue; }
            int cx = std::min(std::max(rx[j],0), w.W-1);
            int cy = std::min(std::max(ry[j],0), w.H-1);
            if (cost_map[size_t(cy)*w.W+cx] >= HI) model_feasible = false;
            double tcv = w.tc(cy,cx);
            double pen = (tcv >= w.rock_sentinel) ? ROCK_PENALTY : tcv;
            if (pen >= HI) rock_hit = true;
        }
    }
    std::vector<double> bare(w.true_cost.size());
    for (size_t i = 0; i < bare.size(); ++i)
        bare[i] = (w.true_cost[i] >= w.rock_sentinel) ? 0.0 : w.true_cost[i];
    double bare_cost = path_cost(w, path, bare);
    double bar = w.optimal_cost * w.tol;
    bool success = start_ok && goal_ok && model_feasible
                   && !rock_hit && !out_of_bounds && bare_cost <= bar;
    std::printf("start_ok=%d goal_ok=%d model_feasible=%d rock_hit=%d "
                "out_of_bounds=%d cost=%.1f (bar=%.1f) -> %s\n",
                start_ok, goal_ok, model_feasible, rock_hit, out_of_bounds,
                bare_cost, bar, success ? "PASS" : "FAIL");
    return success;
}
