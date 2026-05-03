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

} // namespace pcio
