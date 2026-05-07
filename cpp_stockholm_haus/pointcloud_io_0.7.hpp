#pragma once

/*
 * pointcloud_io.hpp — Header-only PCD and PLY loader
 *
 * Supports:
 *   PCD: ASCII and binary (little-endian float/double/int), any field order,
 *        width×height or unorganised layouts, compressed (binary_compressed)
 *        is detected and rejected with a clear error.
 *
 *   PLY: ASCII and binary_little_endian / binary_big_endian,
 *        float/double/int/short/uchar x y z properties,
 *        extra properties (rgb, normal_x, …) are skipped automatically.
 *
 * Usage:
 *   #include "pointcloud_io.hpp"
 *   auto result = pcio::load("scan.pcd");   // or "scan.ply"
 *   if (!result) { std::cerr << result.error << "\n"; return 1; }
 *   auto& pts = result.points;              // std::vector<std::array<float,3>>
 *
 * No dependencies beyond the C++17 standard library.
 */

#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <cassert>
#include <algorithm>
#include <stdexcept>
#include <cctype>
#include <iomanip>
#include <numeric> // added for std::iota

namespace pcio {

using Point3f = std::array<float, 3>;

struct LoadResult {
    std::vector<Point3f> points;
    std::string          error;   // empty on success
    explicit operator bool() const { return error.empty(); }
};

// ─────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────
namespace detail {

// Case-insensitive string compare
inline bool iequal(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (std::tolower((unsigned char)a[i]) != std::tolower((unsigned char)b[i]))
            return false;
    return true;
}

inline std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

inline std::vector<std::string> split(const std::string& s) {
    std::istringstream ss(s);
    std::vector<std::string> tok;
    std::string t;
    while (ss >> t) tok.push_back(t);
    return tok;
}

// Byte-swap helpers for big-endian PLY
inline void swap2(uint8_t* p) { std::swap(p[0], p[1]); }
inline void swap4(uint8_t* p) { std::swap(p[0], p[3]); std::swap(p[1], p[2]); }
inline void swap8(uint8_t* p) { for (int i=0;i<4;++i) std::swap(p[i],p[7-i]); }

// Read exactly n bytes or throw
inline void read_exact(std::ifstream& f, void* buf, size_t n) {
    if (!f.read(reinterpret_cast<char*>(buf), (std::streamsize)n))
        throw std::runtime_error("unexpected end of file");
}

// ─────────────────────────────────────────────
// PCD type descriptors
// ─────────────────────────────────────────────
struct PcdField {
    std::string name;
    char        type;   // F=float I=int U=unsigned
    int         size;   // bytes: 1,2,4,8
    int         count;
};

// Skip 'size' bytes in binary stream
inline void skip_bytes(std::ifstream& f, size_t n) {
    f.seekg((std::streamoff)n, std::ios::cur);
}

inline float read_pcd_scalar(std::ifstream& f, char type, int size, bool is_be) {
    uint8_t buf[8]{};
    read_exact(f, buf, size);
    if (is_be) {
        if (size == 2) swap2(buf);
        else if (size == 4) swap4(buf);
        else if (size == 8) swap8(buf);
    }
    if (type == 'F') {
        if (size == 4) { float v; std::memcpy(&v, buf, 4); return v; }
        if (size == 8) { double v; std::memcpy(&v, buf, 8); return (float)v; }
    } else if (type == 'I') {
        if (size == 1) { int8_t  v; std::memcpy(&v, buf, 1); return (float)v; }
        if (size == 2) { int16_t v; std::memcpy(&v, buf, 2); return (float)v; }
        if (size == 4) { int32_t v; std::memcpy(&v, buf, 4); return (float)v; }
    } else { // U
        if (size == 1) { uint8_t  v; std::memcpy(&v, buf, 1); return (float)v; }
        if (size == 2) { uint16_t v; std::memcpy(&v, buf, 2); return (float)v; }
        if (size == 4) { uint32_t v; std::memcpy(&v, buf, 4); return (float)v; }
    }
    return 0.f;
}

// ─────────────────────────────────────────────
// PCD loader
// ─────────────────────────────────────────────
inline LoadResult load_pcd(std::ifstream& f) {
    LoadResult res;

    std::vector<PcdField> fields;
    int    n_points  = 0;
    bool   is_binary = false;
    bool   is_compressed = false;

    // ── Parse header ──
    std::string line;
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        auto tok = split(line);
        if (tok.empty()) continue;

        std::string key = tok[0];
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c){ return (char)std::toupper(c); });

        if (key == "FIELDS") {
            fields.resize(tok.size() - 1);
            for (size_t i = 1; i < tok.size(); ++i)
                fields[i-1].name = tok[i];
        } else if (key == "SIZE") {
            for (size_t i = 1; i < tok.size() && i-1 < fields.size(); ++i)
                fields[i-1].size = std::stoi(tok[i]);
        } else if (key == "TYPE") {
            for (size_t i = 1; i < tok.size() && i-1 < fields.size(); ++i) {
                char t = (char)std::toupper((unsigned char)tok[i][0]);
                fields[i-1].type = (t == 'F' || t == 'I') ? t : 'U';
            }
        } else if (key == "COUNT") {
            for (size_t i = 1; i < tok.size() && i-1 < fields.size(); ++i)
                fields[i-1].count = std::stoi(tok[i]);
        } else if (key == "POINTS" || key == "WIDTH") {
            // WIDTH × HEIGHT = POINTS for organised; we just use POINTS if set
            if (key == "POINTS")
                n_points = std::stoi(tok[1]);
            // WIDTH sets n_points if POINTS not seen yet and height will be 1
        } else if (key == "HEIGHT") {
            // handled after WIDTH
        } else if (key == "DATA") {
            std::string dt = tok.size() > 1 ? tok[1] : "";
            std::transform(dt.begin(), dt.end(), dt.begin(),
                           [](unsigned char c){ return (char)std::tolower(c); });
            if (dt == "binary_compressed") { is_compressed = true; is_binary = true; }
            else if (dt == "binary")        { is_binary = true; }
            // ascii → defaults (is_binary = false)
            break; // DATA is always last header line
        }
    }

    // If POINTS not explicitly set, figure it out from WIDTH×HEIGHT
    // (already fine: n_points stays 0 if neither seen, we'll read until EOF)
    if (n_points <= 0) {
        res.error = "PCD: could not determine point count from header";
        return res;
    }

    if (is_compressed) {
        res.error = "PCD: binary_compressed format not supported "
                    "(decompress with pcl_convert_pcd_ascii_binary first)";
        return res;
    }

    // Find x/y/z column indices
    int xi=-1, yi=-1, zi=-1;
    for (int i = 0; i < (int)fields.size(); ++i) {
        if (fields[i].name == "x") xi = i;
        else if (fields[i].name == "y") yi = i;
        else if (fields[i].name == "z") zi = i;
    }
    if (xi < 0 || yi < 0 || zi < 0) {
        res.error = "PCD: file has no x/y/z fields";
        return res;
    }

    res.points.reserve(n_points);

    // ── Read points ──
    if (!is_binary) {
        // ASCII
        for (int pt = 0; pt < n_points; ++pt) {
            if (!std::getline(f, line)) break;
            auto tok2 = split(line);
            if ((int)tok2.size() <= std::max({xi,yi,zi})) continue;
            res.points.push_back({std::stof(tok2[xi]),
                                  std::stof(tok2[yi]),
                                  std::stof(tok2[zi])});
        }
    } else {
        // Binary: compute record stride
        int stride = 0;
        for (auto& fld : fields) stride += fld.size * std::max(1, fld.count);

        // Compute byte offsets for x, y, z
        int off_x=0, off_y=0, off_z=0, off=0;
        for (int i = 0; i < (int)fields.size(); ++i) {
            if (i == xi) off_x = off;
            if (i == yi) off_y = off;
            if (i == zi) off_z = off;
            off += fields[i].size * std::max(1, fields[i].count);
        }

        std::vector<uint8_t> rec(stride);
        for (int pt = 0; pt < n_points; ++pt) {
            read_exact(f, rec.data(), stride);

            auto read_field = [&](int byte_off, char type, int sz) -> float {
                uint8_t buf[8]{};
                std::memcpy(buf, rec.data() + byte_off, sz);
                if (type == 'F') {
                    if (sz == 4) { float  v; std::memcpy(&v, buf, 4); return v; }
                    if (sz == 8) { double v; std::memcpy(&v, buf, 8); return (float)v; }
                } else if (type == 'I') {
                    if (sz == 1) { int8_t  v; std::memcpy(&v, buf, 1); return (float)v; }
                    if (sz == 2) { int16_t v; std::memcpy(&v, buf, 2); return (float)v; }
                    if (sz == 4) { int32_t v; std::memcpy(&v, buf, 4); return (float)v; }
                } else {
                    if (sz == 1) { uint8_t  v; std::memcpy(&v, buf, 1); return (float)v; }
                    if (sz == 2) { uint16_t v; std::memcpy(&v, buf, 2); return (float)v; }
                    if (sz == 4) { uint32_t v; std::memcpy(&v, buf, 4); return (float)v; }
                }
                return 0.f;
            };

            res.points.push_back({
                read_field(off_x, fields[xi].type, fields[xi].size),
                read_field(off_y, fields[yi].type, fields[yi].size),
                read_field(off_z, fields[zi].type, fields[zi].size)
            });
        }
    }

    return res;
}

// ─────────────────────────────────────────────
// PLY loader
// ─────────────────────────────────────────────

// Property descriptor for one vertex property
struct PlyProp {
    std::string name;
    int         size;      // bytes: 1,2,4,8
    bool        is_float;  // true = float/double, false = int/uint/uchar/…
    int         col_index; // 0=x, 1=y, 2=z, -1=skip
};

inline LoadResult load_ply(std::ifstream& f) {
    LoadResult res;

    bool ascii  = false;
    bool big_endian = false;
    int  n_vertex = 0;
    bool in_vertex_element = false;
    std::vector<PlyProp> props;

    // ── Parse header ──
    std::string line;
    // First line must be "ply"
    if (!std::getline(f, line) || trim(line) != "ply") {
        res.error = "PLY: magic 'ply' not found";
        return res;
    }

    while (std::getline(f, line)) {
        line = trim(line);
        if (line == "end_header") break;
        auto tok = split(line);
        if (tok.empty()) continue;

        if (tok[0] == "format") {
            if      (tok[1] == "ascii")                    ascii = true;
            else if (tok[1] == "binary_big_endian")        big_endian = true;
            // binary_little_endian → defaults (ascii=false, big_endian=false)
        } else if (tok[0] == "element") {
            in_vertex_element = (tok[1] == "vertex");
            if (in_vertex_element && tok.size() > 2)
                n_vertex = std::stoi(tok[2]);
        } else if (tok[0] == "property" && in_vertex_element) {
            if (tok[1] == "list") continue; // skip list properties
            PlyProp p{};
            std::string type_str = tok[1];
            // Determine size + float-ness
            if      (type_str == "float"  || type_str == "float32") { p.size=4; p.is_float=true; }
            else if (type_str == "double" || type_str == "float64") { p.size=8; p.is_float=true; }
            else if (type_str == "char"   || type_str == "int8")    { p.size=1; p.is_float=false; }
            else if (type_str == "uchar"  || type_str == "uint8")   { p.size=1; p.is_float=false; }
            else if (type_str == "short"  || type_str == "int16")   { p.size=2; p.is_float=false; }
            else if (type_str == "ushort" || type_str == "uint16")  { p.size=2; p.is_float=false; }
            else if (type_str == "int"    || type_str == "int32")   { p.size=4; p.is_float=false; }
            else if (type_str == "uint"   || type_str == "uint32")  { p.size=4; p.is_float=false; }
            else { p.size=4; p.is_float=false; } // unknown → skip but stride-correct

            p.name = tok.size() > 2 ? tok[2] : "";
            if      (p.name == "x") p.col_index = 0;
            else if (p.name == "y") p.col_index = 1;
            else if (p.name == "z") p.col_index = 2;
            else                    p.col_index = -1; // skip

            props.push_back(p);
        }
    }

    if (n_vertex <= 0) { res.error = "PLY: vertex count is 0 or not found"; return res; }

    // Check x/y/z present
    bool has_x=false, has_y=false, has_z=false;
    for (auto& p : props) {
        if (p.col_index == 0) has_x = true;
        if (p.col_index == 1) has_y = true;
        if (p.col_index == 2) has_z = true;
    }
    if (!has_x || !has_y || !has_z) {
        res.error = "PLY: file has no x/y/z vertex properties";
        return res;
    }

    res.points.reserve(n_vertex);

    auto decode_val = [&](const uint8_t* buf, const PlyProp& p) -> float {
        if (p.is_float) {
            if (p.size == 4) { float  v; std::memcpy(&v, buf, 4); return v; }
            if (p.size == 8) { double v; std::memcpy(&v, buf, 8); return (float)v; }
        } else {
            if (p.size == 1) { uint8_t  v; std::memcpy(&v, buf, 1); return (float)v; }
            if (p.size == 2) { int16_t  v; std::memcpy(&v, buf, 2); return (float)v; }
            if (p.size == 4) { int32_t  v; std::memcpy(&v, buf, 4); return (float)v; }
        }
        return 0.f;
    };

    if (ascii) {
        for (int i = 0; i < n_vertex; ++i) {
            if (!std::getline(f, line)) break;
            auto tok2 = split(line);
            Point3f pt{};
            for (int j = 0; j < (int)props.size() && j < (int)tok2.size(); ++j) {
                if (props[j].col_index >= 0)
                    pt[props[j].col_index] = std::stof(tok2[j]);
            }
            res.points.push_back(pt);
        }
    } else {
        // Compute stride
        int stride = 0;
        for (auto& p : props) stride += p.size;

        // Byte offsets within the record
        std::vector<int> offsets(props.size());
        int off = 0;
        for (int j = 0; j < (int)props.size(); ++j) {
            offsets[j] = off;
            off += props[j].size;
        }

        std::vector<uint8_t> rec(stride);
        for (int i = 0; i < n_vertex; ++i) {
            read_exact(f, rec.data(), stride);

            // Byte-swap if big-endian
            if (big_endian) {
                for (int j = 0; j < (int)props.size(); ++j) {
                    uint8_t* p = rec.data() + offsets[j];
                    if      (props[j].size == 2) swap2(p);
                    else if (props[j].size == 4) swap4(p);
                    else if (props[j].size == 8) swap8(p);
                }
            }

            Point3f pt{};
            for (int j = 0; j < (int)props.size(); ++j)
                if (props[j].col_index >= 0)
                    pt[props[j].col_index] = decode_val(rec.data() + offsets[j], props[j]);

            res.points.push_back(pt);
        }
    }

    return res;
}

// ─────────────────────────────────────────────
// Extension detection
// ─────────────────────────────────────────────
inline std::string extension(const std::string& path) {
    auto dot = path.rfind('.');
    if (dot == std::string::npos) return "";
    std::string ext = path.substr(dot + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c){ return (char)std::tolower(c); });
    return ext;
}

} // namespace detail

// ─────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────

/// Load a .pcd or .ply file and return its XYZ points.
inline LoadResult load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {{}, "cannot open file: " + path};

    std::string ext = detail::extension(path);
    try {
        if      (ext == "pcd") return detail::load_pcd(f);
        else if (ext == "ply") return detail::load_ply(f);
        else return {{}, "unsupported extension '" + ext + "' (expected .pcd or .ply)"};
    } catch (const std::exception& e) {
        return {{}, std::string("parse error: ") + e.what()};
    }
}

// ─────────────────────────────────────────────
// Optional writer (ASCII PCD — useful for testing)
// ─────────────────────────────────────────────
inline bool save_pcd_ascii(const std::string& path,
                           const std::vector<Point3f>& pts)
{
    std::ofstream f(path);
    if (!f) return false;
    f << "# .PCD v0.7 - saved by pointcloud_io.hpp\n"
      << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
      << "WIDTH "  << pts.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << pts.size() << "\nDATA ascii\n";
    f << std::setprecision(8) << std::fixed;  // need <iomanip>
    for (auto& p : pts)
        f << p[0] << ' ' << p[1] << ' ' << p[2] << '\n';
    return f.good();
}

// ─────────────────────────────────────────────
// Colored PLY writer — visualise RANSAC results
//
// Each plane gets a distinct color; unassigned points are gray.
// Supports an arbitrary number of planes (colors cycle if >8).
//
// Single-plane convenience overload:
//   pcio::save_colored_ply("out.ply", pts, plane.inliers);
//
// Multi-plane overload:
//   pcio::save_colored_ply("out.ply", pts, {plane0.inliers, plane1.inliers});
// ─────────────────────────────────────────────

struct RGB { uint8_t r, g, b; };

// 8 visually distinct colors (colorblind-friendly-ish)
inline const RGB& plane_color(int plane_idx) {
    static const RGB palette[] = {
        {214,  39,  40},  // red
        { 31, 119, 180},  // blue
        { 44, 160,  44},  // green
        {255, 127,  14},  // orange
        {148, 103, 189},  // purple
        { 23, 190, 207},  // cyan
        {188, 189,  34},  // olive
        {227, 119, 194},  // pink
    };
    return palette[plane_idx % 8];
}

// Binary little-endian PLY with x y z r g b per vertex.
// Format chosen because it's compact, loads in every viewer,
// and Open3D / CloudCompare both read it natively.
inline bool save_colored_ply(
        const std::string& path,
        const std::vector<Point3f>& pts,
        const std::vector<std::vector<uint32_t>>& planes_inliers,
        RGB outlier_color = {120, 120, 120})
{
    // Build per-point color lookup
    std::vector<RGB> colors(pts.size(), outlier_color);
    for (int pi = 0; pi < (int)planes_inliers.size(); ++pi) {
        RGB c = plane_color(pi);
        for (uint32_t idx : planes_inliers[pi]) {
            if (idx < (uint32_t)pts.size()) colors[idx] = c;
        }
    }

    std::ofstream f(path, std::ios::binary);
    if (!f) return false;

    // Header (plain text)
    std::string hdr =
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment saved by pointcloud_io.hpp\n"
        "element vertex " + std::to_string(pts.size()) + "\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n";
    f.write(hdr.data(), (std::streamsize)hdr.size());

    // Data: 15 bytes per point (12 float xyz + 3 uchar rgb)
#pragma pack(push, 1)
    struct Record { float x, y, z; uint8_t r, g, b; };
#pragma pack(pop)
    static_assert(sizeof(Record) == 15, "unexpected struct padding");

    for (size_t i = 0; i < pts.size(); ++i) {
        Record rec{ pts[i][0], pts[i][1], pts[i][2],
                    colors[i].r, colors[i].g, colors[i].b };
        f.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
    }
    return f.good();
}

// Single-plane convenience overload
inline bool save_colored_ply(
        const std::string& path,
        const std::vector<Point3f>& pts,
        const std::vector<uint32_t>& inliers,
        RGB inlier_color  = {214, 39, 40},   // red
        RGB outlier_color = {120, 120, 120})  // gray
{
    std::vector<RGB> colors(pts.size(), outlier_color);
    for (uint32_t idx : inliers)
        if (idx < (uint32_t)pts.size()) colors[idx] = inlier_color;

    std::ofstream f(path, std::ios::binary);
    if (!f) return false;

    std::string hdr =
        "ply\nformat binary_little_endian 1.0\n"
        "comment saved by pointcloud_io.hpp\n"
        "element vertex " + std::to_string(pts.size()) + "\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n";
    f.write(hdr.data(), (std::streamsize)hdr.size());

#pragma pack(push, 1)
    struct Record { float x, y, z; uint8_t r, g, b; };
#pragma pack(pop)

    for (size_t i = 0; i < pts.size(); ++i) {
        Record rec{ pts[i][0], pts[i][1], pts[i][2],
                    colors[i].r, colors[i].g, colors[i].b };
        f.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
    }
    return f.good();
}

// ─────────────────────────────────────────────
// Plane mesh writer
//
// For each detected plane, projects its inlier points onto the plane,
// computes a 2-D convex hull, triangulates it with a fan, then lifts
// the hull vertices back to 3-D.  The result is a watertight polygon
// per plane stored as a PLY face list alongside the original points.
//
// Both point cloud (all input pts) and faces (one per plane) are written
// into a single PLY so every viewer (CloudCompare, MeshLab, Open3D)
// sees the coloured cloud plus the plane polygons in one file.
//
// Usage:
//   // From ransac::Plane objects
//   std::vector<pcio::PlaneDesc> descs;
//   for (auto& p : planes)
//       descs.push_back({ p.normal, p.d, p.inliers });
//   pcio::save_plane_mesh("planes.ply", all_pts, descs);
// ─────────────────────────────────────────────

struct PlaneDesc {
    std::array<float,3>      normal;   // unit normal
    float                    d;        // plane equation: dot(n,p)+d=0
    std::vector<uint32_t>    inliers;  // indices into the global point cloud
};

namespace detail {

// 2-D convex hull (Andrew's monotone chain). Returns indices into pts2d
// in CCW order.
inline std::vector<int>
convex_hull_2d(const std::vector<std::array<float,2>>& pts)
{
    int n = (int)pts.size();
    if (n < 3) {
        std::vector<int> all(n); for (int i=0;i<n;++i) all[i]=i;
        return all;
    }

    // Sort indices by (x, y)
    std::vector<int> idx(n);
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int a, int b){
        return pts[a][0] < pts[b][0] ||
              (pts[a][0] == pts[b][0] && pts[a][1] < pts[b][1]);
    });

    auto cross2 = [&](int O, int A, int B) -> float {
        return (pts[A][0]-pts[O][0])*(pts[B][1]-pts[O][1])
              -(pts[A][1]-pts[O][1])*(pts[B][0]-pts[O][0]);
    };

    std::vector<int> hull;
    hull.reserve(2*n);

    // Lower hull
    for (int i = 0; i < n; ++i) {
        while (hull.size() >= 2 && cross2(hull[hull.size()-2], hull.back(), idx[i]) <= 0)
            hull.pop_back();
        hull.push_back(idx[i]);
    }
    // Upper hull
    int lower_size = (int)hull.size() + 1;
    for (int i = n-2; i >= 0; --i) {
        while ((int)hull.size() >= lower_size && cross2(hull[hull.size()-2], hull.back(), idx[i]) <= 0)
            hull.pop_back();
        hull.push_back(idx[i]);
    }
    hull.pop_back(); // last point == first
    return hull;
}

// Build an orthonormal basis {u, v} on the plane defined by `normal`.
inline void plane_basis(const std::array<float,3>& n,
                        std::array<float,3>& u,
                        std::array<float,3>& v)
{
    // Pick an axis not parallel to n
    std::array<float,3> arb = (std::abs(n[0]) < 0.9f)
                             ? std::array<float,3>{1,0,0}
                             : std::array<float,3>{0,1,0};

    // u = cross(n, arb), v = cross(n, u)
    u = { n[1]*arb[2]-n[2]*arb[1],
          n[2]*arb[0]-n[0]*arb[2],
          n[0]*arb[1]-n[1]*arb[0] };
    float lu = std::sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]);
    u = {u[0]/lu, u[1]/lu, u[2]/lu};

    v = { n[1]*u[2]-n[2]*u[1],
          n[2]*u[0]-n[0]*u[2],
          n[0]*u[1]-n[1]*u[0] };
    float lv = std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    v = {v[0]/lv, v[1]/lv, v[2]/lv};
}

inline float dot3(const std::array<float,3>& a, const std::array<float,3>& b){
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

// ── Binary morphology on a flat occupancy bitmap ─────────────────────
//
// Both operate on a vector<uint8_t> of size nu*nv where 1 = occupied.
// Structuring element: square of radius `r` (side = 2r+1).
// r=1 is a 3×3 SE (standard 4/8-connected morphology).

inline std::vector<uint8_t>
erode_grid(const std::vector<uint8_t>& src, int nu, int nv, int r)
{
    if (r <= 0) return src;
    std::vector<uint8_t> dst(nu * nv, 0);
    for (int j = 0; j < nv; ++j)
    for (int i = 0; i < nu; ++i) {
        if (!src[j*nu+i]) { dst[j*nu+i]=0; continue; }
        bool all = true;
        for (int dj = -r; dj <= r && all; ++dj)
        for (int di = -r; di <= r && all; ++di) {
            int ni = i+di, nj = j+dj;
            if (ni<0||ni>=nu||nj<0||nj>=nv) { all=false; break; }
            if (!src[nj*nu+ni]) all = false;
        }
        dst[j*nu+i] = all ? 1 : 0;
    }
    return dst;
}

inline std::vector<uint8_t>
dilate_grid(const std::vector<uint8_t>& src, int nu, int nv, int r)
{
    if (r <= 0) return src;
    std::vector<uint8_t> dst(nu * nv, 0);
    for (int j = 0; j < nv; ++j)
    for (int i = 0; i < nu; ++i) {
        if (!src[j*nu+i]) continue;
        for (int dj = -r; dj <= r; ++dj)
        for (int di = -r; di <= r; ++di) {
            int ni = i+di, nj = j+dj;
            if (ni>=0&&ni<nu&&nj>=0&&nj<nv)
                dst[nj*nu+ni] = 1;
        }
    }
    return dst;
}

// ── 2-D occupancy grid filter ─────────────────────────────────────────
//
// Projects inlier points onto the plane's UV coordinate system, bins
// them into a regular grid, applies threshold → erode → dilate, then
// keeps only points whose cell is still occupied.
//
// Pipeline:
//   raw counts → threshold (min_pts_per_cell)
//              → erode  (removes thin protrusions and isolated blobs)
//              → dilate (restores surface that was only shaved by erosion)
//              → keep points in surviving cells
//
// Returns the subset of `inlier_indices` that survive the filter.
// Also optionally outputs the intermediate bitmaps for image export.
inline std::vector<uint32_t>
grid_filter_inliers(
        const std::vector<Point3f>&   pts,
        const std::vector<uint32_t>&  inlier_indices,
        const std::array<float,3>&    /*normal*/,
        const std::array<float,3>&    u_ax,
        const std::array<float,3>&    v_ax,
        float  cell_size,
        int    min_pts_per_cell,
        int    erode_iters  = 0,
        int    dilate_iters = 0,
        // optional outputs for image export (may be nullptr)
        int*                        out_nu      = nullptr,
        int*                        out_nv      = nullptr,
        std::vector<int>*           out_counts  = nullptr,
        std::vector<uint8_t>*       out_pre_morph  = nullptr,
        std::vector<uint8_t>*       out_post_morph = nullptr)
{
    if (inlier_indices.empty()) return {};

    // ── Project to 2-D UV ────────────────────────────────────────────
    std::vector<std::array<float,2>> uv(inlier_indices.size());
    float umin =  std::numeric_limits<float>::max();
    float vmin =  std::numeric_limits<float>::max();
    float umax = -std::numeric_limits<float>::max();
    float vmax = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < inlier_indices.size(); ++i) {
        const auto& p = pts[inlier_indices[i]];
        float pu = dot3(p, u_ax);
        float pv = dot3(p, v_ax);
        uv[i] = {pu, pv};
        umin = std::min(umin, pu); umax = std::max(umax, pu);
        vmin = std::min(vmin, pv); vmax = std::max(vmax, pv);
    }

    int nu = std::max(1, (int)std::ceil((umax - umin) / cell_size));
    int nv = std::max(1, (int)std::ceil((vmax - vmin) / cell_size));

    if ((int64_t)nu * nv > 4'000'000)
        return inlier_indices; // grid too large — pass through

    // ── Populate count grid ───────────────────────────────────────────
    std::vector<int> counts(nu * nv, 0);
    for (auto& p2 : uv) {
        int ci = std::min((int)((p2[0]-umin)/cell_size), nu-1);
        int cj = std::min((int)((p2[1]-vmin)/cell_size), nv-1);
        counts[cj * nu + ci]++;
    }

    // ── Threshold → binary bitmap ─────────────────────────────────────
    std::vector<uint8_t> bitmap(nu * nv);
    for (int k = 0; k < nu*nv; ++k)
        bitmap[k] = (counts[k] >= min_pts_per_cell) ? 1 : 0;

    // Expose pre-morph state for image export
    if (out_nu)         *out_nu       = nu;
    if (out_nv)         *out_nv       = nv;
    if (out_counts)     *out_counts   = counts;
    if (out_pre_morph)  *out_pre_morph  = bitmap;

    // ── Morphology: erode then dilate (= opening) ─────────────────────
    // Opening removes isolated specks and thin peninsulas without
    // shrinking the bulk of the surface.
    for (int e = 0; e < erode_iters;  ++e)  bitmap = erode_grid (bitmap, nu, nv, 1);
    for (int d = 0; d < dilate_iters; ++d)  bitmap = dilate_grid(bitmap, nu, nv, 1);

    if (out_post_morph) *out_post_morph = bitmap;

    // ── Keep points in surviving cells ────────────────────────────────
    std::vector<uint32_t> kept;
    kept.reserve(inlier_indices.size());
    for (size_t i = 0; i < inlier_indices.size(); ++i) {
        int ci = std::min((int)((uv[i][0]-umin)/cell_size), nu-1);
        int cj = std::min((int)((uv[i][1]-vmin)/cell_size), nv-1);
        if (bitmap[cj * nu + ci])
            kept.push_back(inlier_indices[i]);
    }
    return kept;
}

} // namespace detail

// ── Grid filter configuration ─────────────────────────────────────────
struct GridFilterConfig {
    bool  enabled          = true;
    float cell_size        = 0.10f; // metres; tune to your scanner resolution
    int   min_pts_per_cell = 2;     // raise for noisier / sparser clouds
    int   erode_iters      = 1;     // opening pass: removes speck artifacts
    int   dilate_iters     = 1;     // re-expand after erosion (= opening)
    // Set erode_iters > dilate_iters for net shrink (conservative boundary).
    // Set dilate_iters > erode_iters for net grow  (closing, fills small holes).
};


// ── Grid image export ─────────────────────────────────────────────────
//
// Writes one PNG image per plane showing the occupancy grid.
// Each pixel = one grid cell. Color encodes point count:
//
//   black          — empty cell (0 pts)
//   dark→bright    — occupied cells, brightness ∝ log(count+1)
//   red border     — cells that would be REMOVED by min_pts_per_cell
//   green border   — cells that survive the filter
//
// A second "diff" image is written alongside showing kept (green) vs
// removed (red) cells against the raw density, making it trivial to
// spot whether your threshold is cutting real surface or just noise.
//
// Output filenames: {prefix}_plane0_density.png, _plane0_diff.png, …
//
// Usage:
//   pcio::GridImageConfig img_cfg;
//   img_cfg.cell_size        = 0.10f;
//   img_cfg.min_pts_per_cell = 3;
//   img_cfg.max_image_dim    = 1024;   // longest axis capped (pixels = cells)
//   pcio::save_grid_images("debug/plane", pts, descs, img_cfg);
// ─────────────────────────────────────────────────────────────────────

struct GridImageConfig {
    float cell_size        = 0.10f;
    int   min_pts_per_cell = 2;
    int   erode_iters      = 1;
    int   dilate_iters     = 1;
    int   max_image_dim    = 2048;
    bool  write_density    = true; // raw point-count heatmap
    bool  write_diff       = true; // threshold kept(green)/removed(red)
    bool  write_morph      = true; // post-erosion+dilation result
};

namespace detail {

// ── Minimal PNG writer (no libpng) ────────────────────────────────────
// Writes 8-bit RGB PNG using only zlib's DEFLATE with stored blocks
// (compression level 0 — fast, ~0% overhead vs raw).
// Compatible with all PNG readers.

// CRC-32 table
inline uint32_t crc32_byte(uint32_t crc, uint8_t b) {
    crc ^= b;
    for (int i = 0; i < 8; ++i)
        crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    return crc;
}
inline uint32_t crc32_buf(const uint8_t* d, size_t n, uint32_t crc = 0xFFFFFFFFu) {
    for (size_t i = 0; i < n; ++i) crc = crc32_byte(crc, d[i]);
    return crc ^ 0xFFFFFFFFu;
}

// Adler-32 for zlib
inline uint32_t adler32_buf(const uint8_t* d, size_t n, uint32_t s = 1) {
    uint32_t s1 = s & 0xFFFF, s2 = s >> 16;
    for (size_t i = 0; i < n; ++i) {
        s1 = (s1 + d[i]) % 65521;
        s2 = (s2 + s1)   % 65521;
    }
    return (s2 << 16) | s1;
}

// Write big-endian u32
inline void w32be(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back((v>>24)&0xFF); out.push_back((v>>16)&0xFF);
    out.push_back((v>> 8)&0xFF); out.push_back( v     &0xFF);
}
inline void w16be(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back((v>>8)&0xFF); out.push_back(v&0xFF);
}
inline void w16le(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(v&0xFF); out.push_back((v>>8)&0xFF);
}

inline void png_chunk(std::vector<uint8_t>& out,
                      const char type[4],
                      const std::vector<uint8_t>& data)
{
    w32be(out, (uint32_t)data.size());
    size_t type_off = out.size();
    out.insert(out.end(), type, type+4);
    out.insert(out.end(), data.begin(), data.end());
    uint32_t crc = crc32_buf(out.data() + type_off, 4 + data.size());
    w32be(out, crc);
}

// Encode raw scanline data as a zlib stream with stored (uncompressed) blocks.
// scanlines: height rows, each = [filter_byte(0), r,g,b, r,g,b, …]
inline std::vector<uint8_t>
zlib_stored(const std::vector<uint8_t>& raw)
{
    // zlib header: CMF=0x78 (deflate, window=32k), FLG computed for no dict
    std::vector<uint8_t> zlib;
    zlib.push_back(0x78);
    zlib.push_back(0x01); // level 0 (fastest), check bits

    // DEFLATE stored blocks: max 65535 bytes each
    const size_t BLOCK = 65535;
    size_t pos = 0;
    while (pos < raw.size()) {
        size_t len = std::min(BLOCK, raw.size() - pos);
        bool   last = (pos + len >= raw.size());
        zlib.push_back(last ? 0x01 : 0x00); // BFINAL | BTYPE=00
        w16le(zlib, (uint16_t)len);
        w16le(zlib, (uint16_t)(~len));
        zlib.insert(zlib.end(), raw.data()+pos, raw.data()+pos+len);
        pos += len;
    }

    uint32_t adc = adler32_buf(raw.data(), raw.size());
    w32be(zlib, adc);
    return zlib;
}

// rgb: width*height*3 bytes, row-major, top-to-bottom
inline bool write_png(const std::string& path,
                      int w, int h,
                      const std::vector<uint8_t>& rgb)
{
    std::vector<uint8_t> png;
    // Signature
    const uint8_t sig[] = {137,80,78,71,13,10,26,10};
    png.insert(png.end(), sig, sig+8);

    // IHDR
    std::vector<uint8_t> ihdr;
    w32be(ihdr, (uint32_t)w);
    w32be(ihdr, (uint32_t)h);
    ihdr.push_back(8);  // bit depth
    ihdr.push_back(2);  // color type: RGB
    ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);
    png_chunk(png, "IHDR", ihdr);

    // Build raw scanlines (filter byte 0 per row)
    std::vector<uint8_t> raw;
    raw.reserve((size_t)(w*3+1)*h);
    for (int y = 0; y < h; ++y) {
        raw.push_back(0); // filter None
        raw.insert(raw.end(),
                   rgb.data() + (size_t)y*w*3,
                   rgb.data() + (size_t)y*w*3 + w*3);
    }

    // IDAT
    png_chunk(png, "IDAT", zlib_stored(raw));

    // IEND
    png_chunk(png, "IEND", {});

    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(reinterpret_cast<const char*>(png.data()), (std::streamsize)png.size());
    return f.good();
}

// ── Heatmap colormap: black → dark blue → cyan → yellow → white ──────
// log-scaled so sparse cells are still visible
inline void density_color(int count, int max_count, uint8_t& r, uint8_t& g, uint8_t& b) {
    if (count == 0) { r=g=b=0; return; }
    float t = std::log((float)count + 1.f) / std::log((float)max_count + 1.f);
    t = std::max(0.f, std::min(1.f, t));
    // 4-stop colormap: black(0) → blue(0.25) → cyan(0.5) → yellow(0.75) → white(1)
    struct Stop { float t; uint8_t r,g,b; };
    static const Stop stops[] = {
        {0.00f,   0,  0,  0},
        {0.25f,   0,  0,200},
        {0.50f,   0,200,200},
        {0.75f, 220,220,  0},
        {1.00f, 255,255,255},
    };
    int i = 0;
    while (i < 3 && stops[i+1].t <= t) ++i;
    float s = (t - stops[i].t) / (stops[i+1].t - stops[i].t + 1e-9f);
    s = std::max(0.f, std::min(1.f, s));
    r = (uint8_t)(stops[i].r + s*(stops[i+1].r - stops[i].r));
    g = (uint8_t)(stops[i].g + s*(stops[i+1].g - stops[i].g));
    b = (uint8_t)(stops[i].b + s*(stops[i+1].b - stops[i].b));
}

} // namespace detail


// ── Public grid image export ───────────────────────────────────────────
struct GridImageStats {
    int   n_images;         // number of PNG files written
    int   n_planes;         // number of planes processed
    bool  ok;
};

inline GridImageStats
save_grid_images(const std::string&            prefix,
                 const std::vector<Point3f>&   pts,
                 const std::vector<PlaneDesc>& planes,
                 const GridImageConfig&        cfg = {})
{
    GridImageStats stats{};
    stats.n_planes = (int)planes.size();

    // Helper: render a bitmap as an image.
    // mode 0 = density heatmap, mode 1 = diff (pre-morph), mode 2 = morph overlay
    auto render = [&](const std::string& path,
                      int nu, int nv,
                      const std::vector<int>&     counts,
                      const std::vector<uint8_t>& pre,   // threshold bitmap
                      const std::vector<uint8_t>& post,  // post-morph bitmap
                      int mode) -> bool
    {
        int max_count = counts.empty() ? 1
                      : *std::max_element(counts.begin(), counts.end());
        if (max_count == 0) max_count = 1;

        std::vector<uint8_t> rgb((size_t)nu * nv * 3, 0);

        for (int cj = 0; cj < nv; ++cj)
        for (int ci = 0; ci < nu; ++ci) {
            int k   = cj*nu + ci;
            size_t off = ((size_t)(nv-1-cj)*nu + ci)*3; // flip V
            int cnt = k < (int)counts.size() ? counts[k] : 0;
            float t = std::log((float)cnt+1.f) / std::log((float)max_count+1.f);
            uint8_t bright = (uint8_t)(80 + 175*t);

            if (mode == 0) {
                // Density heatmap
                detail::density_color(cnt, max_count,
                                      rgb[off], rgb[off+1], rgb[off+2]);
            } else if (mode == 1) {
                // Pre-morph diff: green=kept by threshold, red=removed
                if (cnt == 0) continue;
                bool kept = k < (int)pre.size() && pre[k];
                if (kept) { rgb[off]=0;      rgb[off+1]=bright; rgb[off+2]=0; }
                else      { rgb[off]=bright; rgb[off+1]=0;      rgb[off+2]=0; }
            } else {
                // Morph overlay — three categories:
                //   cyan   = survived threshold AND morphology  (core surface)
                //   yellow = added back by dilation             (restored)
                //   red    = removed by erosion                 (artifacts)
                //   dark   = empty / below threshold
                if (cnt == 0) continue;
                bool was = k < (int)pre.size()  && pre[k];
                bool now = k < (int)post.size() && post[k];
                if  (was && now) {
                    // kept throughout — cyan, brightness by density
                    rgb[off]=0; rgb[off+1]=(uint8_t)(bright*0.9f); rgb[off+2]=bright;
                } else if (!was && now) {
                    // dilated back in — yellow
                    rgb[off]=bright; rgb[off+1]=bright; rgb[off+2]=0;
                } else if (was && !now) {
                    // eroded away — red
                    rgb[off]=bright; rgb[off+1]=0; rgb[off+2]=0;
                }
                // was=false, now=false (below threshold) → stays black
            }
        }
        return detail::write_png(path, nu, nv, rgb);
    };

    for (int pi = 0; pi < (int)planes.size(); ++pi) {
        const PlaneDesc& pd = planes[pi];
        if (pd.inliers.empty()) continue;

        std::array<float,3> u_ax, v_ax;
        detail::plane_basis(pd.normal, u_ax, v_ax);

        // Run the full grid filter pipeline, capturing intermediate bitmaps.
        // We scale cell_size up if the raw grid would exceed max_image_dim,
        // so the images aren't enormous.
        float eff_cell = cfg.cell_size;
        {
            // Quick extent calculation to check grid size
            float umin= 1e30f, umax=-1e30f, vmin= 1e30f, vmax=-1e30f;
            for (uint32_t idx : pd.inliers) {
                if (idx >= (uint32_t)pts.size()) continue;
                float pu = detail::dot3(pts[idx], u_ax);
                float pv = detail::dot3(pts[idx], v_ax);
                umin=std::min(umin,pu); umax=std::max(umax,pu);
                vmin=std::min(vmin,pv); vmax=std::max(vmax,pv);
            }
            int nu_raw = std::max(1,(int)std::ceil((umax-umin)/eff_cell));
            int nv_raw = std::max(1,(int)std::ceil((vmax-vmin)/eff_cell));
            int dim    = std::max(nu_raw, nv_raw);
            if (dim > cfg.max_image_dim)
                eff_cell *= (float)dim / cfg.max_image_dim;
        }

        int                 nu=0, nv=0;
        std::vector<int>    counts;
        std::vector<uint8_t> pre_morph, post_morph;

        detail::grid_filter_inliers(
            pts, pd.inliers, pd.normal, u_ax, v_ax,
            eff_cell, cfg.min_pts_per_cell,
            cfg.erode_iters, cfg.dilate_iters,
            &nu, &nv, &counts, &pre_morph, &post_morph);

        if (nu == 0 || nv == 0) continue;

        std::string base = prefix + "_plane" + std::to_string(pi);

        if (cfg.write_density)
            if (render(base+"_density.png", nu,nv, counts, pre_morph, post_morph, 0))
                stats.n_images++;

        if (cfg.write_diff)
            if (render(base+"_diff.png",    nu,nv, counts, pre_morph, post_morph, 1))
                stats.n_images++;

        if (cfg.write_morph)
            if (render(base+"_morph.png",   nu,nv, counts, pre_morph, post_morph, 2))
                stats.n_images++;
    }

    stats.ok = (stats.n_images > 0 || planes.empty());
    return stats;
}


struct MeshStats {
    size_t n_vertices;       // total vertices written (pts + hull verts)
    size_t n_faces;          // total triangles written
    size_t n_planes_written; // planes that produced valid hulls
    bool   ok;
};

// ── Main writer ───────────────────────────────────────────────────────
// Writes a single PLY containing:
//   • All cloud points (coloured by plane membership, gray = outlier)
//   • One convex-hull polygon per plane, triangulated as a fan,
//     with per-face color matching the palette.
//
// `alpha_pct` (0–255) sets the face color's opacity in viewers that
// support it (stored as a fourth uchar channel).  Most viewers ignore
// it but it doesn't break loading.
inline MeshStats
save_plane_mesh(const std::string&            path,
                const std::vector<Point3f>&   pts,
                const std::vector<PlaneDesc>& planes,
                RGB              outlier_color = {120, 120, 120},
                uint8_t          alpha_pct     = 180,
                GridFilterConfig grid          = {})
{
    MeshStats stats{};

    // ── 1. Apply grid filter + build per-point colors ─────────────────
    // We run the filter first so filtered-out inliers are colored gray,
    // not the plane color — giving immediate visual feedback on what was removed.
    std::vector<RGB> pt_colors(pts.size(), outlier_color);

    // filtered_inliers[pi] = surviving inlier indices after grid filter
    std::vector<std::vector<uint32_t>> filtered_inliers(planes.size());

    for (int pi = 0; pi < (int)planes.size(); ++pi) {
        const PlaneDesc& pd = planes[pi];

        std::array<float,3> u, v;
        detail::plane_basis(pd.normal, u, v);

        // Apply grid filter (or pass through if disabled)
        if (grid.enabled && pd.inliers.size() >= 3) {
            filtered_inliers[pi] = detail::grid_filter_inliers(
                pts, pd.inliers, pd.normal, u, v,
                grid.cell_size, grid.min_pts_per_cell,
                grid.erode_iters, grid.dilate_iters);
        } else {
            filtered_inliers[pi] = pd.inliers;
        }

        RGB c = plane_color(pi);
        for (uint32_t idx : filtered_inliers[pi])
            if (idx < (uint32_t)pts.size()) pt_colors[idx] = c;
        // Note: inliers removed by the grid filter stay outlier_color
    }

    // ── 2. Build hull geometry for each plane ─────────────────────────
    struct HullVert { float x,y,z; RGB col; };
    struct HullTri  { uint32_t a,b,c; RGB col; };

    std::vector<HullVert> hull_verts;
    std::vector<HullTri>  hull_tris;

    // Hull vertices are appended after the cloud points.
    uint32_t vert_offset = (uint32_t)pts.size();

    for (int pi = 0; pi < (int)planes.size(); ++pi) {
        const PlaneDesc& pd = planes[pi];
        const auto& kept = filtered_inliers[pi];
        if (kept.size() < 3) continue;

        std::array<float,3> u, v;
        detail::plane_basis(pd.normal, u, v);

        // Project surviving inliers onto plane's 2-D coordinate system
        std::vector<std::array<float,2>> pts2d;
        pts2d.reserve(kept.size());
        for (uint32_t idx : kept) {
            if (idx >= (uint32_t)pts.size()) continue;
            const auto& p = pts[idx];
            pts2d.push_back({ detail::dot3(p,u), detail::dot3(p,v) });
        }

        if (pts2d.size() < 3) continue;

        // Convex hull in 2-D
        auto hull_idx = detail::convex_hull_2d(pts2d);
        if (hull_idx.size() < 3) continue;

        // Compute centroid on the plane (for fan triangulation pivot)
        float cx=0,cy=0,cz=0;
        for (uint32_t idx : kept) {
            if (idx < (uint32_t)pts.size()) {
                cx+=pts[idx][0]; cy+=pts[idx][1]; cz+=pts[idx][2];
            }
        }
        float inv = 1.f / (float)kept.size();
        cx *= inv; cy *= inv; cz *= inv;
        // Snap centroid onto the plane: c' = c - dot(n,c+d)*n
        float dist = pd.normal[0]*cx + pd.normal[1]*cy + pd.normal[2]*cz + pd.d;
        float pcx = cx - pd.normal[0]*dist;
        float pcy = cy - pd.normal[1]*dist;
        float pcz = cz - pd.normal[2]*dist;

        RGB col = plane_color(pi);

        // Centroid vertex (fan pivot) — snapped to plane surface
        uint32_t center_vi = vert_offset + (uint32_t)hull_verts.size();
        hull_verts.push_back({pcx, pcy, pcz, col});

        // Hull ring vertices.
        // UV coords in pts2d are absolute (dot(p, u/v_axis)).
        // We lift back using the snapped centroid as the plane origin so the
        // result is independent of any error in d:
        //   world = snapped_centroid + (pu - pu_centroid)*u + (pv - pv_centroid)*v
        float pu_c = detail::dot3({pcx,pcy,pcz}, u);
        float pv_c = detail::dot3({pcx,pcy,pcz}, v);

        uint32_t ring_start = vert_offset + (uint32_t)hull_verts.size();
        for (int hi : hull_idx) {
            float du = pts2d[hi][0] - pu_c;
            float dv = pts2d[hi][1] - pv_c;
            hull_verts.push_back({
                pcx + du*u[0] + dv*v[0],
                pcy + du*u[1] + dv*v[1],
                pcz + du*u[2] + dv*v[2],
                col});
        }

        // Fan triangles: center + consecutive hull edge
        uint32_t ring_n = (uint32_t)hull_idx.size();
        for (uint32_t hi = 0; hi < ring_n; ++hi) {
            uint32_t a = center_vi;
            uint32_t b = ring_start + hi;
            uint32_t c = ring_start + (hi+1) % ring_n;
            hull_tris.push_back({a, b, c, col});
        }

        stats.n_planes_written++;
    }

    // ── 3. Write PLY ──────────────────────────────────────────────────
    size_t total_verts = pts.size() + hull_verts.size();
    size_t total_faces = hull_tris.size();

    std::ofstream f(path, std::ios::binary);
    if (!f) return stats;

    // Header
    std::string hdr =
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment plane mesh saved by pointcloud_io.hpp\n"
        "element vertex " + std::to_string(total_verts) + "\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "element face " + std::to_string(total_faces) + "\n"
        "property list uchar uint vertex_indices\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "property uchar alpha\n"
        "end_header\n";
    f.write(hdr.data(), (std::streamsize)hdr.size());

    // Cloud vertices
#pragma pack(push, 1)
    struct VRec { float x,y,z; uint8_t r,g,b; };
#pragma pack(pop)
    static_assert(sizeof(VRec)==15,"");

    for (size_t i = 0; i < pts.size(); ++i) {
        VRec rec{pts[i][0], pts[i][1], pts[i][2],
                 pt_colors[i].r, pt_colors[i].g, pt_colors[i].b};
        f.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
    }

    // Hull vertices
    for (auto& hv : hull_verts) {
        VRec rec{hv.x, hv.y, hv.z, hv.col.r, hv.col.g, hv.col.b};
        f.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
    }

    // Faces: uchar count (3) + 3×uint32 indices + rgba
#pragma pack(push, 1)
    struct FRec { uint8_t cnt; uint32_t i0,i1,i2; uint8_t r,g,b,a_ch; };
#pragma pack(pop)
    static_assert(sizeof(FRec)==17,"");

    for (auto& t : hull_tris) {
        FRec rec{3, t.a, t.b, t.c, t.col.r, t.col.g, t.col.b, alpha_pct};
        f.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
    }

    stats.n_vertices = total_verts;
    stats.n_faces    = total_faces;
    stats.ok         = f.good();
    return stats;
}

} // namespace pcio
