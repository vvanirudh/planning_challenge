// PROVIDED planner (do not edit -- just call it). astar_plan returns a
// near-optimal start->goal (row,col) path under the given (H x W) cost_map, found
// by 8-connected grid A*. Cells with cost >= ROCK_PENALTY/2 are treated as
// untraversable. Move cost between adjacent cells = destination cell cost scaled
// by step length (1 or sqrt(2)), matching path_cost's integral.
#pragma once
#include "metric_p1.hpp"
#include "terrain_p1.hpp"
#include <cmath>
#include <queue>
#include <tuple>
#include <vector>

inline Path astar_plan(const World& w, const Grid& cost_map) {
    const double HI = ROCK_PENALTY / 2.0;
    auto clampi = [](int v, int lo, int hi){ return std::min(std::max(v,lo),hi); };
    int sr = clampi((int)std::lround(w.start.r), 0, w.H-1);
    int sc = clampi((int)std::lround(w.start.c), 0, w.W-1);
    int gr = clampi((int)std::lround(w.goal.r),  0, w.H-1);
    int gc = clampi((int)std::lround(w.goal.c),  0, w.W-1);
    auto idx = [&](int r, int c){ return size_t(r)*w.W + c; };
    auto h = [&](int r, int c){ return std::hypot(double(r-gr), double(c-gc)); };

    std::vector<double> g(size_t(w.H)*w.W, INFINITY);
    std::vector<int> came(size_t(w.H)*w.W, -1);
    g[idx(sr,sc)] = 0.0;
    using QE = std::tuple<double,int,int>;   // (f, r, c)
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
    pq.push({h(sr,sc), sr, sc});
    const int dr[8] = {-1,1,0,0,-1,-1,1,1};
    const int dc[8] = {0,0,-1,1,-1,1,-1,1};
    while (!pq.empty()) {
        auto [f, r, c] = pq.top(); pq.pop();
        if (r == gr && c == gc) break;
        double gc_cur = g[idx(r,c)];
        if (f - h(r,c) > gc_cur + 1e-9) continue;   // stale entry
        for (int k = 0; k < 8; ++k) {
            int nr = r + dr[k], nc = c + dc[k];
            if (nr < 0 || nr >= w.H || nc < 0 || nc >= w.W) continue;
            double cell = cost_map[nr][nc];
            if (cell >= HI) continue;               // rock / forbidden
            double ng = gc_cur + cell * std::hypot(double(dr[k]), double(dc[k]));
            if (ng < g[idx(nr,nc)]) {
                g[idx(nr,nc)] = ng;
                came[idx(nr,nc)] = int(idx(r,c));
                pq.push({ng + h(nr,nc), nr, nc});
            }
        }
    }
    if (!std::isfinite(g[idx(gr,gc)])) {
        // No feasible route: return the straight line so the metric can score it.
        Path p(60);
        for (int i = 0; i < 60; ++i) {
            double a = double(i)/59.0;
            p[i] = { w.start.r+(w.goal.r-w.start.r)*a,
                     w.start.c+(w.goal.c-w.start.c)*a };
        }
        return p;
    }
    std::vector<int> cells;
    int cur = int(idx(gr,gc));
    while (cur != -1) { cells.push_back(cur); cur = came[cur]; }
    Path path;
    for (auto it = cells.rbegin(); it != cells.rend(); ++it) {
        int r = *it / w.W, c = *it % w.W;
        path.push_back({ double(r), double(c) });   // (row, col)
    }
    return path;
}
