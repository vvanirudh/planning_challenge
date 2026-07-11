// PRE-BUILT METRIC (do not edit) -- exact port of the Python path_cost /
// evaluate. The true per-step terrain cost is an opaque grid in
// meta['true_cost']; don't peek -- recovering it from the demos is the point.
#pragma once
#include "terrain_p1.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>

constexpr double OOB_PENALTY = 1e6;    // per-sample charge for an out-of-bounds point
constexpr double STEP = 0.5;           // path-integral resampling spacing (cells)

// Resample a (row,col) path to ~uniform STEP spacing; fill rounded int cell
// indices (rows, cols) and an in-bounds flag per sample.
inline bool resample_steps(const World& w, const Path& path,
                           std::vector<int>& rows, std::vector<int>& cols,
                           std::vector<bool>& in_bounds) {
    std::vector<double> seg(path.size() ? path.size()-1 : 0);
    double L = 0;
    for (size_t k = 1; k < path.size(); ++k) {
        double dr = path[k].r-path[k-1].r, dc = path[k].c-path[k-1].c;
        seg[k-1] = std::sqrt(dr*dr+dc*dc); L += seg[k-1];
    }
    if (L < 1e-9) return false;
    std::vector<double> s(path.size());
    s[0] = 0; for (size_t k = 1; k < path.size(); ++k) s[k] = s[k-1]+seg[k-1];
    int n = std::max(2, int(L/STEP) + 1);
    rows.resize(n); cols.resize(n); in_bounds.assign(n, false);
    for (int j = 0; j < n; ++j) {
        double u = L * j / double(n-1);
        size_t k = 1; while (k < s.size()-1 && s[k] < u) ++k;
        double t = (s[k]-s[k-1]>1e-12) ? (u-s[k-1])/(s[k]-s[k-1]) : 0.0;
        double rr = path[k-1].r + (path[k].r-path[k-1].r)*t;
        double cc = path[k-1].c + (path[k].c-path[k-1].c)*t;
        int ir = (int)std::lround(rr), ic = (int)std::lround(cc);
        rows[j] = ir; cols[j] = ic;
        in_bounds[j] = (ir >= 0 && ir <= w.H-1 && ic >= 0 && ic <= w.W-1);
    }
    return true;
}

inline double path_cost(const World& w, const Path& path, const Grid& cost_map) {
    std::vector<int> rows, cols; std::vector<bool> ib;
    if (!resample_steps(w, path, rows, cols, ib)) return INFINITY;
    double total = 0;
    for (size_t j = 0; j < rows.size(); ++j) {
        if (ib[j]) {
            int cr = std::min(std::max(rows[j],0), w.H-1);
            int cc = std::min(std::max(cols[j],0), w.W-1);
            total += cost_map[cr][cc];
        } else total += OOB_PENALTY;
    }
    return total * STEP;
}

// Official PASS/FAIL scorer. Checks (1) endpoints near start/goal, (2) the path
// stays in bounds, and (3) its terrain cost (under the hidden true cost) is
// <= the near-optimal bar (optimal_cost * tol). `cost_map` is accepted for
// backwards compatibility but unused -- there are no impassable cells, so scoring
// depends only on the true terrain cost. Prints a one-line summary.
inline bool evaluate(const World& w, const Path& path, const Grid& /*cost_map*/ = {}) {
    double d0r = path.front().r-w.start.r, d0c = path.front().c-w.start.c;
    double d1r = path.back().r -w.goal.r,  d1c = path.back().c -w.goal.c;
    bool start_ok = std::sqrt(d0r*d0r+d0c*d0c) < 5;
    bool goal_ok  = std::sqrt(d1r*d1r+d1c*d1c) < 5;

    std::vector<int> rows, cols; std::vector<bool> ib;
    bool out_of_bounds = true;
    if (resample_steps(w, path, rows, cols, ib)) {
        out_of_bounds = false;
        for (size_t j = 0; j < rows.size(); ++j)
            if (!ib[j]) { out_of_bounds = true; break; }
    }
    double cost = path_cost(w, path, w.true_cost);   // true terrain cost along the path
    double bar = w.optimal_cost * w.tol;
    bool success = start_ok && goal_ok && !out_of_bounds && cost <= bar;
    std::printf("start_ok=%d goal_ok=%d out_of_bounds=%d "
                "cost=%.1f (bar=%.1f) -> %s\n",
                start_ok, goal_ok, out_of_bounds, cost, bar,
                success ? "PASS" : "FAIL");
    return success;
}
