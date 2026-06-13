// Domain data: the field, demos, start/goal, metric metadata, and a show()
// that dumps everything needed to plot to a binary file (rendered by the
// Python `show_cpp` matplotlib helper). The data is loaded from `world.bin`,
// which the Python setup cell packs from the .npy/.npz files -- so there is no
// file-format (NPY/NPZ/pickle) parsing on the C++ side at all.
#pragma once
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <vector>

// Channel indices, matching the Python ROCK/MUD/SLOPE = 0/1/2.
enum { ROCK = 0, MUD = 1, SLOPE = 2 };

struct Vec2 { double x = 0, y = 0; };
using Path = std::vector<Vec2>;

struct World {
    int H = 0, W = 0;
    std::vector<float> field;            // (H*W*3) row-major: [r,c,ch]
    std::vector<Path>  demos;            // expert (x,y) polylines
    Vec2 start, goal;
    std::vector<double> true_cost;       // (H*W) opaque scoring grid
    double expert_cost = 0, tol = 0, rock_sentinel = 0;

    float fld(int r, int c, int ch) const { return field[(size_t(r)*W + c)*3 + ch]; }
    double tc(int r, int c) const { return true_cost[size_t(r)*W + c]; }
};

// Read `world.bin` (packed by the Python setup cell). Layout is the single
// shared contract with that cell -- see its comment for the byte layout.
template <class T> static T rd(std::ifstream& f) {
    T v; f.read((char*)&v, sizeof(T)); return v;
}
inline World load_world() {
    std::ifstream f("world.bin", std::ios::binary);
    if (!f) throw std::runtime_error("cannot open world.bin (run the setup cell)");
    World w;
    w.H = rd<int32_t>(f); w.W = rd<int32_t>(f);
    size_t N = size_t(w.H) * w.W;
    w.field.resize(N * 3);
    f.read((char*)w.field.data(), std::streamsize(w.field.size() * sizeof(float)));
    w.true_cost.resize(N);
    f.read((char*)w.true_cost.data(), std::streamsize(N * sizeof(double)));
    w.start = { rd<double>(f), rd<double>(f) };
    w.goal  = { rd<double>(f), rd<double>(f) };
    w.expert_cost   = rd<double>(f);
    w.tol           = rd<double>(f);
    w.rock_sentinel = rd<double>(f);
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

// show(): dump the cost map (default: true_cost), demos, candidate paths, and
// start/goal to a binary file that the Python `show_cpp` helper renders with
// matplotlib (so the C++ side stays tiny and the plot matches the Python
// notebook exactly). Format of viz_<tag>.bin (all little-endian):
//   int32 H, int32 W
//   H*W float64  grid (row-major)
//   H*W uint8    rock mask (1 = rock)
//   double start_x, start_y, goal_x, goal_y
//   int32 n_demos; per demo: int32 T, then T*2 float64 (x,y interleaved)
//   int32 n_paths; per path: int32 T, then T*2 float64 (x,y interleaved)
inline void dump_path(std::ofstream& o, const Path& p) {
    int32_t T = (int32_t)p.size(); o.write((char*)&T, 4);
    for (const auto& q : p) { o.write((char*)&q.x, 8); o.write((char*)&q.y, 8); }
}
inline void show(const World& w, const std::vector<Path>& paths,
                 const std::vector<double>* cost_map, const std::string& out_bin,
                 const std::string& /*title*/ = "") {
    const std::vector<double>& grid = cost_map ? *cost_map : w.true_cost;
    std::ofstream o(out_bin, std::ios::binary);
    int32_t H = w.H, W = w.W;
    o.write((char*)&H, 4); o.write((char*)&W, 4);
    o.write((const char*)grid.data(), std::streamsize(grid.size()*8));
    std::vector<unsigned char> rock(size_t(H)*W);
    for (int r = 0; r < H; ++r) for (int c = 0; c < W; ++c)
        rock[size_t(r)*W + c] = (w.fld(r,c,ROCK) == 1.0) ? 1 : 0;
    o.write((const char*)rock.data(), std::streamsize(rock.size()));
    o.write((char*)&w.start.x, 8); o.write((char*)&w.start.y, 8);
    o.write((char*)&w.goal.x, 8);  o.write((char*)&w.goal.y, 8);
    int32_t nd = (int32_t)w.demos.size(); o.write((char*)&nd, 4);
    for (const auto& d : w.demos) dump_path(o, d);
    int32_t np = (int32_t)paths.size(); o.write((char*)&np, 4);
    for (const auto& p : paths) dump_path(o, p);
}
