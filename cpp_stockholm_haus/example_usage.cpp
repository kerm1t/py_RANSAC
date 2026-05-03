// example_usage.cpp
//
// Compile: g++ -O3 -std=c++17 -o demo example_usage.cpp
//
#include "ransac_plane.hpp"
#include "pointcloud_io.hpp"

#include <iostream>
#include <iomanip>
#include <cassert>
#include <random>
#include <cstdio>

static void run(const std::vector<ransac::Point3f>& pts, const char* label) {
    ransac::Config cfg;
    cfg.distance_threshold = 0.05f;
    cfg.confidence         = 0.999f;
    cfg.n_threads          = std::thread::hardware_concurrency();
    auto p = ransac::fit_plane(pts, cfg);
    std::printf("%-26s  pts=%7zu  ", label, pts.size());
    if (p.valid)
        std::printf("normal=(%.3f,%.3f,%.3f) d=%+.3f  inliers=%zu (%d iters)\n",
                    p.normal[0], p.normal[1], p.normal[2], p.d,
                    p.inliers.size(), p.iterations_run);
    else
        std::printf("no plane found\n");
}

int main(int argc, char** argv) {

    // ── If a file was passed on the command line, load + run ──────────
    if (argc >= 2) {
        auto r = pcio::load(argv[1]);
        if (!r) { std::cerr << "Error: " << r.error << '\n'; return 1; }
        run(r.points, argv[1]);
        return 0;
    }

    // ── Self-contained demo ───────────────────────────────────────────
    std::mt19937 rng(0);
    std::uniform_real_distribution<float> uni(-4.f, 4.f);
    std::normal_distribution<float> noise(0.f, 0.01f);

    std::vector<ransac::Point3f> cloud;

    // Floor tilted 30° around Y
    float a = std::sin(30.f * 3.14159265f / 180.f);
    float c = std::cos(30.f * 3.14159265f / 180.f);
    for (int i = 0; i < 5000; ++i) {
        float u=uni(rng), v=uni(rng), n=noise(rng);
        cloud.push_back({u*c + n*a, v, -u*a + n*c});
    }
    // Vertical wall
    for (int i = 0; i < 2000; ++i)
        cloud.push_back({uni(rng), 3.f + noise(rng), uni(rng)});
    // Outliers
    for (int i = 0; i < 500; ++i)
        cloud.push_back({uni(rng), uni(rng), uni(rng)});

    // ── Fit planes ───────────────────────────────────────────────────
    ransac::Config cfg;
    cfg.distance_threshold = 0.05f;
    cfg.confidence         = 0.999f;
    cfg.n_threads          = std::thread::hardware_concurrency();

    auto planes = ransac::fit_planes(cloud, 3, cfg, 0.05f);
    std::printf("Found %zu plane(s)\n", planes.size());
    for (int i=0;i<(int)planes.size();++i)
        std::printf("  [%d] n=(%.3f,%.3f,%.3f)  d=%+.3f  inliers=%zu\n",
                    i, planes[i].normal[0],planes[i].normal[1],planes[i].normal[2],
                    planes[i].d, planes[i].inliers.size());

    // ── Align to axes + origin ────────────────────────────────────────
    std::puts("\n--- Aligning to axes ---");
    auto aligned = ransac::align_to_axes_and_origin(cloud, planes);

    const char* axname[]={"X","Y","Z"};
    for (int i=0;i<(int)aligned.planes.size();++i){
        int ai = aligned.axis_assignment[i];
        std::printf("  plane %d  normal=(%.4f,%.4f,%.4f)  assigned_axis=%s\n",
                    i, aligned.planes[i].normal[0],
                    aligned.planes[i].normal[1],
                    aligned.planes[i].normal[2],
                    ai>=0 ? axname[ai] : "none");
    }

    // ── Save raw and aligned mesh PLYs ────────────────────────────────
    {
        std::vector<pcio::PlaneDesc> descs;
        for (auto& p : planes)
            descs.push_back({p.normal, p.d, p.inliers});
        pcio::GridFilterConfig gf; gf.cell_size=0.25f;
        auto s = pcio::save_plane_mesh("raw.ply", cloud, descs,{120,120,120},180,gf);
        std::printf("\nraw.ply:     verts=%zu faces=%zu\n", s.n_vertices, s.n_faces);
    }
    {
        std::vector<pcio::PlaneDesc> descs;
        for (auto& p : aligned.planes)
            descs.push_back({p.normal, p.d, p.inliers});
        pcio::GridFilterConfig gf; gf.cell_size=0.25f;
        auto s = pcio::save_plane_mesh("aligned.ply", aligned.points, descs,{120,120,120},180,gf);
        std::printf("aligned.ply: verts=%zu faces=%zu\n", s.n_vertices, s.n_faces);
    }

    return 0;
}
