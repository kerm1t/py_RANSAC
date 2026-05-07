#pragma once

/*
 * ransac_plane.hpp — Fast header-only RANSAC plane detector
 *
 * Usage:
 *   std::vector<std::array<float,3>> pts = ...;
 *   auto result = ransac::fit_plane(pts);
 *   // result.normal, result.d, result.inliers
 *
 * Options via ransac::Config.
 */

#include <array>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <algorithm>
#include <numeric>
#include <thread>
#include <mutex>
#include <atomic>
#include <cassert>

namespace ransac {

// ─────────────────────────────────────────────
// Types
// ─────────────────────────────────────────────

using Point3f = std::array<float, 3>;

struct Plane {
    Point3f normal{};   // unit normal
    float   d{};        // signed distance: dot(normal, p) + d = 0
    std::vector<uint32_t> inliers;
    int     iterations_run{};
    bool    valid{false};
};

struct Config {
    float    distance_threshold = 0.02f;  // inlier max distance to plane (metres)
    float    confidence         = 0.999f; // probability of finding a good plane
    float    inlier_ratio_hint  = 0.5f;   // initial guess for inlier ratio w
    int      max_iterations     = 1000;   // hard cap
    int      min_iterations     = 10;
    bool     refit_with_inliers = true;   // SVD refit on final inlier set
    unsigned seed               = 42;
    int      n_threads          = 1;      // set >1 for parallel RANSAC
};

// ─────────────────────────────────────────────
// Tiny 3-D math helpers (no Eigen needed)
// ─────────────────────────────────────────────

namespace detail {

inline Point3f sub(const Point3f& a, const Point3f& b) {
    return {a[0]-b[0], a[1]-b[1], a[2]-b[2]};
}
inline Point3f cross(const Point3f& u, const Point3f& v) {
    return { u[1]*v[2]-u[2]*v[1],
             u[2]*v[0]-u[0]*v[2],
             u[0]*v[1]-u[1]*v[0] };
}
inline float dot(const Point3f& a, const Point3f& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
inline float norm(const Point3f& a) {
    return std::sqrt(dot(a, a));
}
inline Point3f normalize(const Point3f& a) {
    float n = norm(a);
    if (n < 1e-9f) return {0,0,0};
    return {a[0]/n, a[1]/n, a[2]/n};
}

// Plane from 3 non-collinear points. Returns false if degenerate.
inline bool plane_from_3pts(const Point3f& p0,
                             const Point3f& p1,
                             const Point3f& p2,
                             Point3f& normal, float& d)
{
    Point3f n = cross(sub(p1, p0), sub(p2, p0));
    float len = norm(n);
    if (len < 1e-9f) return false;       // collinear / duplicate
    n = {n[0]/len, n[1]/len, n[2]/len};
    normal = n;
    d = -dot(n, p0);
    return true;
}

inline float pt_to_plane_dist(const Point3f& p,
                               const Point3f& n, float d)
{
    return std::abs(dot(n, p) + d);
}

// ─────────────────────────────────────────────
// Adaptive iteration count
// N = log(1−confidence) / log(1 − w^3)
// ─────────────────────────────────────────────
inline int adaptive_n(float confidence, float inlier_ratio) {
    float w3 = inlier_ratio * inlier_ratio * inlier_ratio;
    if (w3 >= 1.0f) return 1;
    float denom = std::log(1.0f - w3);
    if (denom >= 0.0f) return 1000;
    int n = static_cast<int>(std::ceil(std::log(1.0f - confidence) / denom));
    return std::max(1, n);
}

// ─────────────────────────────────────────────
// Collect inlier indices for a given plane
// ─────────────────────────────────────────────
inline std::vector<uint32_t>
collect_inliers(const std::vector<Point3f>& pts,
                const Point3f& normal, float d,
                float threshold)
{
    std::vector<uint32_t> idx;
    idx.reserve(pts.size() / 2);
    for (uint32_t i = 0; i < (uint32_t)pts.size(); ++i)
        if (pt_to_plane_dist(pts[i], normal, d) < threshold)
            idx.push_back(i);
    return idx;
}

// ─────────────────────────────────────────────
// Optional: refit plane via mean + covariance (no Eigen).
// Uses the covariance matrix + power iteration to get the
// dominant eigenvector (fastest axis = normal of flat plane).
// Good enough for most use-cases; replace with full SVD for
// very curved/noisy inlier sets.
// ─────────────────────────────────────────────
inline void refit_plane(const std::vector<Point3f>& pts,
                        const std::vector<uint32_t>& inliers,
                        Point3f& normal, float& d)
{
    if (inliers.size() < 3) return;

    // Centroid
    double cx = 0, cy = 0, cz = 0;
    for (uint32_t i : inliers) {
        cx += pts[i][0]; cy += pts[i][1]; cz += pts[i][2];
    }
    double n = (double)inliers.size();
    cx /= n; cy /= n; cz /= n;

    // 3×3 covariance (upper triangle)
    double cxx=0,cxy=0,cxz=0,cyy=0,cyz=0,czz=0;
    for (uint32_t i : inliers) {
        double dx = pts[i][0]-cx, dy = pts[i][1]-cy, dz = pts[i][2]-cz;
        cxx+=dx*dx; cxy+=dx*dy; cxz+=dx*dz;
        cyy+=dy*dy; cyz+=dy*dz; czz+=dz*dz;
    }

    // Power iteration on the *smallest* eigenvector of C
    // (normal to the plane is the direction of least variance).
    // We negate C so smallest becomes largest, then iterate.
    // C is 3×3 symmetric. We subtract λ_max*I so the smallest
    // eigenvalue's vector is the new dominant one.
    // Simple approach: smallest eigenvalue via inverse power on C.
    // We do 30 iterations of (C + shift*I)^{-1} v using Cramer's rule.

    // Estimate largest eigenvalue via a few power steps
    double trace = cxx + cyy + czz;
    double shift = trace + 1.0; // > max eigenvalue

    // Shifted matrix M = shift*I - C (positive definite → dominant ev = normal)
    // M * v uses Cramer for solve, 20 iters
    Point3f v = normal; // warm start
    double vx = v[0], vy = v[1], vz = v[2];

    for (int iter = 0; iter < 25; ++iter) {
        // M * [vx,vy,vz] where M = shift*I - C
        double nx2 = (shift-cxx)*vx - cxy*vy - cxz*vz;
        double ny2 =        -cxy*vx + (shift-cyy)*vy - cyz*vz;
        double nz2 =        -cxz*vx - cyz*vy + (shift-czz)*vz;
        double len2 = std::sqrt(nx2*nx2 + ny2*ny2 + nz2*nz2);
        if (len2 < 1e-12) break;
        vx = nx2/len2; vy = ny2/len2; vz = nz2/len2;
    }

    normal = normalize({(float)vx, (float)vy, (float)vz});
    // Ensure consistent orientation (flip toward original normal)
    if (dot(normal, {(float)vx,(float)vy,(float)vz}) < 0)
        normal = {-normal[0], -normal[1], -normal[2]};

    d = -(float)(normal[0]*cx + normal[1]*cy + normal[2]*cz);
}

// ─────────────────────────────────────────────
// Single-thread RANSAC core
// ─────────────────────────────────────────────
inline Plane ransac_single(const std::vector<Point3f>& pts,
                            const Config& cfg,
                            unsigned seed)
{
    const int N = (int)pts.size();
    assert(N >= 3);

    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(0, N - 1);

    Plane best;
    best.inliers.reserve(N / 4);

    float w = cfg.inlier_ratio_hint;
    int max_iter = std::min(cfg.max_iterations,
                            std::max(cfg.min_iterations,
                                     adaptive_n(cfg.confidence, w)));

    for (int iter = 0; iter < max_iter; ++iter) {
        // Sample 3 distinct random points
        int i0 = dist(rng), i1, i2;
        do { i1 = dist(rng); } while (i1 == i0);
        do { i2 = dist(rng); } while (i2 == i0 || i2 == i1);

        Point3f normal; float d;
        if (!plane_from_3pts(pts[i0], pts[i1], pts[i2], normal, d))
            continue;

        // Count inliers (hot loop — kept minimal)
        int count = 0;
        float thr = cfg.distance_threshold;
        for (const auto& p : pts)
            count += (pt_to_plane_dist(p, normal, d) < thr) ? 1 : 0;

        if (count > (int)best.inliers.size()) {
            best.normal   = normal;
            best.d        = d;
            best.inliers  = collect_inliers(pts, normal, d, thr);

            // Update adaptive iteration limit
            float new_w = (float)count / N;
            if (new_w > w) {
                w = new_w;
                int new_max = adaptive_n(cfg.confidence, w);
                max_iter = std::min(cfg.max_iterations,
                                    std::max(max_iter, new_max));
            }
        }
        best.iterations_run = iter + 1;
    }

    return best;
}

} // namespace detail

// ─────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────

inline Plane fit_plane(const std::vector<Point3f>& pts,
                       const Config& cfg = {})
{
    if ((int)pts.size() < 3) return {};

    Plane result;

    if (cfg.n_threads <= 1) {
        result = detail::ransac_single(pts, cfg, cfg.seed);
    } else {
        // Parallel: each thread runs its own RANSAC, we take the best.
        std::vector<Plane> thread_results(cfg.n_threads);
        std::vector<std::thread> threads;
        threads.reserve(cfg.n_threads);

        for (int t = 0; t < cfg.n_threads; ++t) {
            unsigned s = cfg.seed + (unsigned)t * 1000001u;
            threads.emplace_back([&, t, s]() {
                thread_results[t] = detail::ransac_single(pts, cfg, s);
            });
        }
        for (auto& th : threads) th.join();

        result = *std::max_element(
            thread_results.begin(), thread_results.end(),
            [](const Plane& a, const Plane& b){
                return a.inliers.size() < b.inliers.size();
            });
    }

    if (result.inliers.empty()) return result;

    if (cfg.refit_with_inliers)
        detail::refit_plane(pts, result.inliers, result.normal, result.d);

    // Recompute final inlier set after refit
    result.inliers = detail::collect_inliers(
        pts, result.normal, result.d, cfg.distance_threshold);
    result.valid = !result.inliers.empty();
    return result;
}

// ─────────────────────────────────────────────
// Iterative multi-plane extraction
// Removes inliers of each detected plane and repeats.
// ─────────────────────────────────────────────
inline std::vector<Plane>
fit_planes(const std::vector<Point3f>& pts,
           int max_planes,
           const Config& cfg = {},
           float min_inlier_fraction = 0.02f)
{
    std::vector<Point3f> remaining = pts;
    std::vector<Plane>   planes;

    // We need to track original indices through removals
    std::vector<uint32_t> global_idx(pts.size());
    std::iota(global_idx.begin(), global_idx.end(), 0u);

    for (int k = 0; k < max_planes; ++k) {
        if ((int)remaining.size() < 3) break;

        Plane p = fit_plane(remaining, cfg);
        if (!p.valid) break;
        if ((float)p.inliers.size() / (float)pts.size() < min_inlier_fraction)
            break;

        // Remap inlier indices to original cloud
        for (auto& idx : p.inliers) idx = global_idx[idx];
        planes.push_back(std::move(p));

        // Remove inlier points from remaining set
        std::vector<Point3f>   next_pts;
        std::vector<uint32_t>  next_idx;
        next_pts.reserve(remaining.size());
        next_idx.reserve(remaining.size());

        // Build a mask
        std::vector<bool> is_inlier(remaining.size(), false);
        for (uint32_t i : planes.back().inliers)
            ; // already remapped; rebuild from plane fit
        // Re-detect inliers from the local 'remaining' indices:
        for (uint32_t li = 0; li < (uint32_t)remaining.size(); ++li) {
            float dist = detail::pt_to_plane_dist(
                remaining[li], planes.back().normal, planes.back().d);
            if (dist >= cfg.distance_threshold) {
                next_pts.push_back(remaining[li]);
                next_idx.push_back(global_idx[li]);
            }
        }
        remaining  = std::move(next_pts);
        global_idx = std::move(next_idx);
    }

    return planes;
}

} // namespace ransac
