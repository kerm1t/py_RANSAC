// example_usage.cpp — demonstrates ransac_plane.hpp
//
// Compile: g++ -O3 -std=c++17 -o demo example_usage.cpp
//
#include "ransac_plane.hpp"

#include <iostream>
#include <cmath>

int main() {
    // ── Build a synthetic cloud: mostly planar + 20% noise ──
    std::mt19937 rng(0);
    std::uniform_real_distribution<float> uni(-5.f, 5.f);
    std::uniform_real_distribution<float> noise(-0.5f, 0.5f);

    std::vector<ransac::Point3f> cloud;
    cloud.reserve(10000);

    // Ground plane at z = 0, normal (0,0,1)
    for (int i = 0; i < 8000; ++i)
        cloud.push_back({uni(rng), uni(rng), 0.01f * noise(rng)});

    // Outliers / second plane
    for (int i = 0; i < 2000; ++i)
        cloud.push_back({uni(rng), uni(rng), uni(rng)});

    // ── Single plane ──
    ransac::Config cfg;
    cfg.distance_threshold = 0.05f;
    cfg.confidence         = 0.999f;
    cfg.n_threads          = std::thread::hardware_concurrency();
    cfg.refit_with_inliers = true;

    auto plane = ransac::fit_plane(cloud, cfg);

    if (plane.valid) {
        auto& n = plane.normal;
        std::printf("Plane  normal: (%.4f, %.4f, %.4f)  d=%.4f\n",
                    n[0], n[1], n[2], plane.d);
        std::printf("Inliers: %zu / %zu  (iterations: %d)\n",
                    plane.inliers.size(), cloud.size(), plane.iterations_run);
    } else {
        std::puts("No plane found.");
    }

    // ── Multi-plane extraction ──
    auto planes = ransac::fit_planes(cloud, 3, cfg, 0.05f);
    std::printf("\nFound %zu plane(s):\n", planes.size());
    for (int i = 0; i < (int)planes.size(); ++i) {
        auto& p = planes[i];
        std::printf("  [%d] normal=(%.3f,%.3f,%.3f) d=%.3f  inliers=%zu\n",
                    i, p.normal[0], p.normal[1], p.normal[2],
                    p.d, p.inliers.size());
    }

    return 0;
}
