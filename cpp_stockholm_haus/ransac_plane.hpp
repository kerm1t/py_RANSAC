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

        Plane p = fit_plane(remaining, cfg); // fit plane <-- RANSAC
        if (!p.valid) break;
        if ((float)p.inliers.size() / (float)pts.size() < min_inlier_fraction) // e.g. less than 5% (0.05) of original points -> dismiss
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

// ─────────────────────────────────────────────────────────────────────
// Axis-alignment: rotate the point cloud so that detected plane normals
// align with the nearest cardinal axis (+X, +Y, or +Z).
//
// Strategy
// ────────
// Given N detected planes, we pick the one whose normal is closest to
// any axis, compute the rotation that maps that normal exactly onto its
// target axis, then cascade: apply the rotation to all remaining plane
// normals and repeat until every plane has been aligned or no unambiguous
// axis assignment remains.
//
// The final rotation R is the composition of all per-step rotations.
// Applying R to every point gives a cloud where:
//   • The dominant plane lies flat (normal → nearest axis).
//   • Secondary planes snap to the remaining axes if their normals are
//     close enough (within `axis_snap_deg` of a remaining axis).
//
// The rotation is a pure SO(3) matrix (no translation, no scale).
// Translation can be added separately (e.g. translate the centroid of
// the largest plane inlier set to the origin).
//
// Usage:
//   auto planes = ransac::fit_planes(pts, 3, cfg);
//   auto result = ransac::align_to_axes(pts, planes);
//   // result.points   — rotated cloud
//   // result.rotation — 3×3 column-major rotation matrix
//   // result.planes   — planes with updated normals + d values
// ─────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────
// Alignment math helpers — kept inside detail:: (already open above)
// ─────────────────────────────────────────────────────────────────────

namespace detail {

using Mat3 = std::array<float, 9>; // column-major: M[col*3+row]

inline Mat3 mat3_identity() {
    return {1,0,0, 0,1,0, 0,0,1};
}

inline Point3f mat3_mul_vec(const Mat3& M, const Point3f& v) {
    return {
        M[0]*v[0] + M[3]*v[1] + M[6]*v[2],
        M[1]*v[0] + M[4]*v[1] + M[7]*v[2],
        M[2]*v[0] + M[5]*v[1] + M[8]*v[2]
    };
}

inline Mat3 mat3_mul(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int col = 0; col < 3; ++col)
        for (int row = 0; row < 3; ++row)
            for (int k = 0; k < 3; ++k)
                C[col*3+row] += A[k*3+row] * B[col*3+k];
    return C;
}

inline Mat3 rotation_matrix(const Point3f& ax, float angle) {
    float c = std::cos(angle), s = std::sin(angle), t = 1.f - c;
    float x=ax[0], y=ax[1], z=ax[2];
    return {
        t*x*x+c,   t*x*y+s*z, t*x*z-s*y,
        t*x*y-s*z, t*y*y+c,   t*y*z+s*x,
        t*x*z+s*y, t*y*z-s*x, t*z*z+c
    };
}

inline Mat3 rotation_between(const Point3f& from, const Point3f& to) {
    float d = dot(from, to);
    if (d >  1.f - 1e-6f) return mat3_identity();
    if (d < -1.f + 1e-6f) {
        Point3f perp = (std::abs(from[0]) < 0.9f)
                     ? normalize(cross(from, {1,0,0}))
                     : normalize(cross(from, {0,1,0}));
        return rotation_matrix(perp, 3.14159265f);
    }
    Point3f ax  = normalize(cross(from, to));
    float   ang = std::acos(std::max(-1.f, std::min(1.f, d)));
    return rotation_matrix(ax, ang);
}

} // namespace detail


// ── Result of align_to_axes ───────────────────────────────────────────
struct AlignResult {
    std::vector<Point3f> points;   // rotated cloud (same size as input)
    detail::Mat3         rotation; // 3×3 column-major SO(3) matrix (R*x = rotated x)
    std::vector<Plane>   planes;   // planes with updated normals and d values
    // For each plane: axis_index = 0→X, 1→Y, 2→Z, -1→unassigned
    std::vector<int>     axis_assignment;
};

// ── align_to_axes ─────────────────────────────────────────────────────
// Aligns each detected plane's normal to its nearest cardinal axis.
// Planes are processed in descending inlier-count order so the largest
// plane drives the primary assignment.  All assigned planes land exactly
// on their axes simultaneously.
//
// Method — orthonormal frame construction:
//   The assigned (source normal → target cardinal axis) pairs define a
//   rigid rotation.  We build an explicit orthonormal source frame S and
//   target frame T from the assigned pairs, then R = T * S^T maps every
//   source column to its target column in one shot — no incremental drift,
//   no frame-mismatch, every assigned plane exact.
//
//   Frame construction with k assigned pairs (k = 1, 2, or 3):
//     k=1: simple rotation_between(n0, axis0)
//     k≥2: S = [s0 | s1 | s0×s1],  T = [t0 | t1 | t0×t1]
//          then R = T * S^T = outer-product sum  sum_i (t_i ⊗ s_i)
inline AlignResult
align_to_axes(const std::vector<Point3f>& pts,
              const std::vector<Plane>&   planes,
              float axis_snap_deg = 45.f)
{
    AlignResult result;
    result.planes          = planes;
    result.rotation        = detail::mat3_identity();
    result.axis_assignment.assign(planes.size(), -1);

    if (planes.empty()) return result;

    const float snap_cos = std::cos(axis_snap_deg * 3.14159265f / 180.f);
    const Point3f E[3] = { {1,0,0}, {0,1,0}, {0,0,1} };

    // ── 1. Greedy axis assignment in inlier-count order ───────────────
    //    All comparisons against original unrotated normals and cardinal axes.
    std::vector<int> order(planes.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b){
        return planes[a].inliers.size() > planes[b].inliers.size();
    });

    std::array<bool,3> used = { false, false, false };
    for (int pi : order) {
        int   best_ai  = -1;
        float best_cos = snap_cos;
        for (int ai = 0; ai < 3; ++ai) {
            if (used[ai]) continue;
            float c = std::abs(planes[pi].normal[ai]); // dot(n, E[ai]) = n[ai]
            if (c > best_cos) { best_cos = c; best_ai = ai; }
        }
        if (best_ai < 0) continue;
        result.axis_assignment[pi] = best_ai;
        used[best_ai] = true;
    }

    // ── 2. Collect source/target column pairs (in inlier-count order) ──
    struct Pair { Point3f s; Point3f t; };
    std::vector<Pair> pairs;
    for (int pi : order) {
        int ai = result.axis_assignment[pi];
        if (ai < 0) continue;
        Point3f t = E[ai];
        if (planes[pi].normal[ai] < 0) t[ai] = -1.f;  // match sign
        pairs.push_back({ planes[pi].normal, t });
        if ((int)pairs.size() == 3) break;
    }

    // ── 3. Build rotation R ────────────────────────────────────────────
    if (pairs.empty()) {
        // No assignment — keep identity
    } else if (pairs.size() == 1) {
        result.rotation = detail::rotation_between(pairs[0].s, pairs[0].t);
    } else {
        // Build orthonormal source frame S = [s0 | s1_orth | s0×s1_orth]
        // and target frame            T = [t0 | t1_orth | t0×t1_orth]
        // then R = sum_k (t_k ⊗ s_k^T)  i.e. R * s_k = t_k for each k
        Point3f s0 = detail::normalize(pairs[0].s);
        Point3f t0 = detail::normalize(pairs[0].t);

        // Orthogonalise s1 against s0
        auto orth = [&](const Point3f& v, const Point3f& ref) -> Point3f {
            float p = detail::dot(v, ref);
            return detail::normalize({v[0]-p*ref[0], v[1]-p*ref[1], v[2]-p*ref[2]});
        };
        Point3f s1 = orth(pairs[1].s, s0);
        Point3f t1 = orth(pairs[1].t, t0);
        Point3f s2 = detail::normalize(detail::cross(s0, s1));
        Point3f t2 = detail::normalize(detail::cross(t0, t1));

        // R = T * S^T = sum_k t_k ⊗ s_k  (outer product)
        // R[row][col] = sum_k t_k[row] * s_k[col]
        // column-major storage: R[col*3+row]
        detail::Mat3& R = result.rotation;
        R = {};
        const Point3f* S[3] = {&s0, &s1, &s2};
        const Point3f* T[3] = {&t0, &t1, &t2};
        for (int k = 0; k < 3; ++k)
            for (int row = 0; row < 3; ++row)
                for (int col = 0; col < 3; ++col)
                    R[col*3+row] += (*T[k])[row] * (*S[k])[col];
    }

    // ── 4. Apply rotation to all points ───────────────────────────────
    result.points.resize(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
        result.points[i] = detail::mat3_mul_vec(result.rotation, pts[i]);

    // ── 5. Recompute normals and d from rotated inlier points ──────────
    for (int pi = 0; pi < (int)result.planes.size(); ++pi) {
        auto& p = result.planes[pi];
        p.normal = detail::normalize(
            detail::mat3_mul_vec(result.rotation, planes[pi].normal));
        if (p.inliers.empty()) continue;
        double sum = 0;
        for (uint32_t idx : p.inliers) {
            if (idx >= (uint32_t)result.points.size()) continue;
            sum += p.normal[0]*result.points[idx][0]
                 + p.normal[1]*result.points[idx][1]
                 + p.normal[2]*result.points[idx][2];
        }
        p.d = -(float)(sum / p.inliers.size());
    }

    return result;
}

// ── Convenience: align + translate so the largest plane sits at origin ─
// The dominant plane (most inliers) is moved to z=0 (or whichever axis
// it was assigned to), centering the cloud on that surface.
inline AlignResult
align_to_axes_and_origin(const std::vector<Point3f>& pts,
                         const std::vector<Plane>&   planes,
                         float axis_snap_deg = 45.f)
{
    auto result = align_to_axes(pts, planes, axis_snap_deg);
    if (result.points.empty()) return result;

    // Find the plane with the most inliers that got an axis assignment
    int dominant = -1;
    for (int pi = 0; pi < (int)result.planes.size(); ++pi) {
        if (result.axis_assignment[pi] < 0) continue;
        if (dominant < 0 ||
            result.planes[pi].inliers.size() > result.planes[dominant].inliers.size())
            dominant = pi;
    }
    if (dominant < 0) return result;

    // Compute the 3-D centroid of the dominant plane's inlier points
    // (using the already-rotated coordinates in result.points).
    double cx = 0, cy = 0, cz = 0;
    const auto& inliers = result.planes[dominant].inliers;
    for (uint32_t idx : inliers) {
        if (idx >= (uint32_t)result.points.size()) continue;
        cx += result.points[idx][0];
        cy += result.points[idx][1];
        cz += result.points[idx][2];
    }
    float inv = 1.f / (float)inliers.size();
    Point3f centroid{ (float)(cx*inv), (float)(cy*inv), (float)(cz*inv) };

    // "selbst geschrieben"
    // min oder max ist hier nicht so klar, je nachdem, wie das gebäude gedreht ist
    // Idee: mit Hilfe von Ground die up/down Orientierung ermitteln 
    float max_y = std::numeric_limits<float>::min();
    for (uint32_t idx : inliers) {
        if (idx >= (uint32_t)result.points.size()) continue;
        max_y = std::max(max_y, result.points[idx][1]);
    }

    // After axis alignment the dominant plane's normal is exactly ±axis,
    // so the centroid already sits on the plane surface.
    // Translate the entire cloud so this centroid moves to the origin.
    Point3f shift{ -centroid[0], -centroid[1], -centroid[2] };

    for (auto& p : result.points) {
        p[0] += shift[0];
// hack!!        p[1] += shift[1];
//        p[1] -= shift[1]/6.0f; // shift up so the floor plane (dominant) sits at y=0 instead of z=0
        p[1] -= max_y; // shift up so the floor plane (dominant) sits at y=0 instead of z=0
        p[2] += shift[2];
    }

    // Update d for every plane by re-measuring from the actual inlier points.
    // This is numerically exact regardless of how d was estimated before,
    // and correctly handles all planes (not just the dominant one).
    for (auto& p : result.planes) {
        if (p.inliers.empty()) continue;
        double sum = 0;
        for (uint32_t idx : p.inliers)
            sum += p.normal[0]*result.points[idx][0]
                 + p.normal[1]*result.points[idx][1]
                 + p.normal[2]*result.points[idx][2];
        p.d = -(float)(sum / p.inliers.size());
    }

    return result;
}

// ── rotate_around_axis ────────────────────────────────────────────────
//
// Apply one or more 90° rotations around a cardinal axis to an existing
// AlignResult, in-place.  Useful to correct the residual in-plane
// orientation after align_to_axes / align_to_axes_and_origin.
//
// Parameters
// ──────────
// result      : AlignResult from align_to_axes[_and_origin]; modified in place
// axis        : 0=X, 1=Y, 2=Z
// steps       : number of 90° steps, positive=CCW, negative=CW when
//               looking from the positive end of the axis toward origin
//               (right-hand rule).  Range −3 … +3 covers all distinct
//               orientations; values outside that range are wrapped.
//
// The function:
//   • builds the exact 90°-step rotation matrix (no floating-point trig)
//   • multiplies it into result.rotation
//   • rotates all points
//   • updates plane normals and d values
//
// Example — flip the cloud 180° around Z after aligning:
//   ransac::rotate_around_axis(aligned, 2, 2);
//
// Example — rotate 90° CCW around Y:
//   ransac::rotate_around_axis(aligned, 1, 1);
// ─────────────────────────────────────────────────────────────────────

inline void rotate_around_axis(AlignResult& result, int axis, int steps)
{
    // Normalise steps to {0,1,2,3}
    steps = ((steps % 4) + 4) % 4;
    if (steps == 0) return;

    // Exact 90°-step rotation matrices (column-major, no trig).
    // Each entry is R_k = (R_90)^k  for k = 1, 2, 3.
    // R_90 around X:  y→z→-y→-z, x fixed
    // R_90 around Y:  z→x→-z→-x, y fixed
    // R_90 around Z:  x→y→-x→-y, z fixed
    //
    // Stored as [k=1, k=2, k=3] per axis.
    // col-major: M[col*3+row]
    //                          col0          col1          col2
    static const detail::Mat3 rot_table[3][3] = {
        // axis=X
        { { 1, 0, 0,   0, 0, 1,   0,-1, 0 },   // 90°  CCW around X
          { 1, 0, 0,   0,-1, 0,   0, 0,-1 },   // 180°
          { 1, 0, 0,   0, 0,-1,   0, 1, 0 } }, // 270° CCW = 90° CW
        // axis=Y
        { { 0, 0,-1,   0, 1, 0,   1, 0, 0 },   // 90°  CCW around Y
          {-1, 0, 0,   0, 1, 0,   0, 0,-1 },   // 180°
          { 0, 0, 1,   0, 1, 0,  -1, 0, 0 } }, // 270°
        // axis=Z
        { { 0, 1, 0,  -1, 0, 0,   0, 0, 1 },   // 90°  CCW around Z
          {-1, 0, 0,   0,-1, 0,   0, 0, 1 },   // 180°
          { 0,-1, 0,   1, 0, 0,   0, 0, 1 } }  // 270°
    };

    if (axis < 0 || axis > 2) return;
    const detail::Mat3& R_step = rot_table[axis][steps - 1];

    // Accumulate
    result.rotation = detail::mat3_mul(R_step, result.rotation);

    // Rotate all points
    for (auto& p : result.points)
        p = detail::mat3_mul_vec(R_step, p);

    // Update plane normals and recompute d from inlier points
    for (auto& p : result.planes) {
        p.normal = detail::mat3_mul_vec(R_step, p.normal);
        p.normal = detail::normalize(p.normal);
        if (p.inliers.empty()) continue;
        double sum = 0;
        for (uint32_t idx : p.inliers)
            sum += p.normal[0]*result.points[idx][0]
                 + p.normal[1]*result.points[idx][1]
                 + p.normal[2]*result.points[idx][2];
        p.d = -(float)(sum / p.inliers.size());
    }
}

} // namespace ransac
