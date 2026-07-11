// Domain data: the features tensor, demos, start/goal, metric metadata, and a
// show() that dumps everything to a binary file (rendered by the Python
// `show_cpp` helper). Loaded from `world_p1.bin`, which the Python setup cell
// packs -- so there is no file-format parsing on the C++ side.
//
// Convention: points are (row, col). A cost map is indexed cost_map[row][col];
// the features tensor is indexed features[row][col][channel]. Every cell is
// traversable at a finite cost -- there are no rocks / impassable cells.
#pragma once
#include <array>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <string>

// Channel indices, matching the Python MUD/SLOPE = 0/1.
enum { MUD = 0, SLOPE = 1 };

struct Vec2 { double r = 0, c = 0; };       // (row, col)
using Path = std::vector<Vec2>;
using Grid = std::vector<std::vector<double>>;                 // (H x W) cost map
using Features = std::vector<std::vector<std::array<float, 2>>>;  // (H x W x 2)

struct World {
    int H = 0, W = 0;
    Features features;                   // features[r][c] = {mud, slope}
    std::vector<Path> demos;             // expert (row,col) polylines
    Vec2 start, goal;                    // (row, col)
    Grid true_cost;                      // opaque scoring grid (finite everywhere)
    double optimal_cost = 0, tol = 0;
};

template <class T> static T rd(std::ifstream& f) {
    T v; f.read((char*)&v, sizeof(T)); return v;
}
inline World load_world() {
    std::ifstream f("world_p1.bin", std::ios::binary);
    if (!f) throw std::runtime_error("cannot open world_p1.bin (run the setup cell)");
    World w;
    w.H = rd<int32_t>(f); w.W = rd<int32_t>(f);
    size_t N = size_t(w.H) * w.W;
    // field arrives flat row-major [r,c,ch]; reshape into the nested tensor.
    std::vector<float> flat(N * 2);
    f.read((char*)flat.data(), std::streamsize(flat.size() * sizeof(float)));
    w.features.assign(w.H, std::vector<std::array<float,2>>(w.W));
    for (int r = 0; r < w.H; ++r) for (int c = 0; c < w.W; ++c)
        for (int ch = 0; ch < 2; ++ch)
            w.features[r][c][ch] = flat[(size_t(r)*w.W + c)*2 + ch];
    // true_cost arrives flat row-major; reshape too.
    std::vector<double> tcflat(N);
    f.read((char*)tcflat.data(), std::streamsize(N * sizeof(double)));
    w.true_cost.assign(w.H, std::vector<double>(w.W));
    for (int r = 0; r < w.H; ++r) for (int c = 0; c < w.W; ++c)
        w.true_cost[r][c] = tcflat[size_t(r)*w.W + c];
    w.start = { rd<double>(f), rd<double>(f) };   // (row, col)
    w.goal  = { rd<double>(f), rd<double>(f) };
    w.optimal_cost = rd<double>(f);
    w.tol          = rd<double>(f);
    int32_t nd = rd<int32_t>(f);
    w.demos.resize(nd);
    for (int i = 0; i < nd; ++i) {
        int32_t T = rd<int32_t>(f);
        Path p(T);
        for (int k = 0; k < T; ++k) p[k] = { rd<double>(f), rd<double>(f) };
        w.demos[i] = std::move(p);
    }
    return w;
}

// show(): dump a cost map (default: true_cost), demos, candidate paths, and
// start/goal to a binary file that the Python `show_cpp` helper renders. The grid
// is written flat row-major; points are written (row, col).
inline void dump_path(std::ofstream& o, const Path& p) {
    int32_t T = (int32_t)p.size(); o.write((char*)&T, 4);
    for (const auto& q : p) { o.write((char*)&q.r, 8); o.write((char*)&q.c, 8); }
}
inline void show(const World& w, const std::vector<Path>& paths,
                 const Grid* cost_map, const std::string& out_bin,
                 const std::string& /*title*/ = "") {
    const Grid& grid = cost_map ? *cost_map : w.true_cost;
    std::ofstream o(out_bin, std::ios::binary);
    int32_t H = w.H, W = w.W;
    o.write((char*)&H, 4); o.write((char*)&W, 4);
    for (int r = 0; r < H; ++r)
        o.write((const char*)grid[r].data(), std::streamsize(size_t(W)*8));
    o.write((char*)&w.start.r, 8); o.write((char*)&w.start.c, 8);
    o.write((char*)&w.goal.r, 8);  o.write((char*)&w.goal.c, 8);
    int32_t nd = (int32_t)w.demos.size(); o.write((char*)&nd, 4);
    for (const auto& d : w.demos) dump_path(o, d);
    int32_t np = (int32_t)paths.size(); o.write((char*)&np, 4);
    for (const auto& p : paths) dump_path(o, p);
}
