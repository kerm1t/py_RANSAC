#include "pointcloud_io.hpp"
#include "ransac_plane.hpp"

// this on works nicely for a single plane, but multi-plane extraction is more interesting
/*int main_single_plane() {
  auto cloud = pcio::load("d:\\stockholm.pcd");   // or .ply
  //if (!cloud) { std::cerr << cloud.error; return 1; }
  auto plane = ransac::fit_plane(cloud.points);
  pcio::save_colored_ply("result.ply", cloud.points, plane.inliers);
}
*/
int main() {
////  auto cloud = pcio::load("d:\\stockholm.pcd");   // or .ply
///  auto cloud = pcio::load(".\\stockholm.ply");   // or .pcd
//  auto cloud = pcio::load("C:\\DATA\\SCaffee_data\\beirut\\hibollah_building.ply");   // or .pcd
//  auto cloud = pcio::load("C:\\DATA\\SCaffee_data\\beirut_pi3\\pi3_beirut_hisbollah_building.ply");   // or .pcd
///  auto cloud = pcio::load("C:\\DATA\\SCaffee_data\\beirut_pi3\\pi3_beirut_hisbollah_building_cropped.ply");   // or .pcd
  auto cloud = pcio::load("C:\\DATA\\SCaffee_data\\beirut_pi3\\pi3_beirut_hisbollah_building_cropped_cars.ply");   // or .pcd

   ransac::Config cfg;
// vggt
//    cfg.distance_threshold = 0.004f,//0.05f;
// pi3
    cfg.distance_threshold = 0.02f;
    cfg.confidence         = 0.5f;//999f;
    cfg.n_threads          = std::thread::hardware_concurrency();
    cfg.refit_with_inliers = true;

  auto planes = ransac::fit_planes(cloud.points, 3, cfg, 0.05f);
    std::printf("\nFound %zu plane(s):\n", planes.size());
    for (int i = 0; i < (int)planes.size(); ++i) {
        auto& p = planes[i];
        std::printf("  [%d] normal=(%.3f,%.3f,%.3f) d=%.3f  inliers=%zu\n",
                    i, p.normal[0], p.normal[1], p.normal[2],
                    p.d, p.inliers.size());
    }

  // (6)
  // result.points          — rotated cloud
  // result.rotation        — 3×3 column-major SO(3) matrix
  // result.planes          — planes with updated normals and d
  // result.axis_assignment — per-plane: 0=X, 1=Y, 2=Z, -1=unassigned
// 5/5/2026
  std::vector<std::vector<uint32_t>> facades = {planes[0].inliers, planes[1].inliers}; // habe ich nur die wände gewählt
///  auto result = ransac::align_to_axes(cloud.points, planes);
  auto result = ransac::align_to_axes_and_origin(cloud.points, planes);
////  std::vector<std::vector<uint32_t>> facades = {result.planes[0].inliers, result.planes[1].inliers}; // habe ich nur die wände gewählt

// hack for stockholm building:
ransac::rotate_around_axis(result, 0, -1);  // 90° CW around X

pcio::save_colored_ply("aligned.ply", result.points, facades);

    std::printf("\nAdjusted plane(s) to axes:\n");
    for (int i = 0; i < (int)result.planes.size(); ++i) {
        auto& p = result.planes[i];
        std::printf("  [%d] normal=(%.3f,%.3f,%.3f) d=%.3f  inliers=%zu\n",
                    i, p.normal[0], p.normal[1], p.normal[2],
                    p.d, p.inliers.size());
    }

//  std::vector<std::vector<uint32_t>> all = {planes[0].inliers, planes[1].inliers, planes[2].inliers};
//  pcio::save_colored_ply("result.ply", cloud.points, all);

  std::vector<pcio::PlaneDesc> descs;
//  for (auto& p : planes)
  for (auto& p : result.planes)
      descs.push_back({p.normal, p.d, p.inliers});

/// (3)
//  auto stats = pcio::save_plane_mesh("planes.ply", cloud.points, descs);

// (4) uv
/*  pcio::GridFilterConfig gf;
  gf.cell_size        = 0.01f;//0.10f;  // metres — match your scanner density
  gf.min_pts_per_cell = 12;//2;      // raise for noisier clouds

  auto stats = pcio::save_plane_mesh("out.ply", cloud.points, descs,
                                   {120,120,120}, 180, gf);


// (5) added output of grids as png in order to tune parameters
  pcio::GridImageConfig img;
  img.cell_size        = 0.004f;//0.10f;
  img.min_pts_per_cell = 3;
  img.max_image_dim    = 1024;  // downscale huge planes automatically
  img.write_density    = true;  // black→blue→cyan→yellow→white heatmap
  img.write_diff       = true;  // green=kept, red=removed by threshold

// linux  auto s = pcio::save_grid_images("debug/plane", cloud.points, descs, img);
  auto s = pcio::save_grid_images("plane", cloud.points, descs, img);
  // writes: debug/plane_plane0_density.png
  //         debug/plane_plane0_diff.png
  //         debug/plane_plane1_density.png  …
  printf("%d images written\n", s.n_images);
  */
  pcio::GridFilterConfig gf;
  gf.cell_size        = 0.01f;//0.25f;
  gf.min_pts_per_cell = 2;
  gf.erode_iters      = 1;   // opening: remove artifacts
  gf.dilate_iters     = 1;
  // erode > dilate → net shrink (conservative boundary)
  // dilate > erode → net grow  (closing, fills small holes)

//5/5/2026
//pcio::save_plane_mesh("out.ply", cloud.points, descs, {120,120,120}, 180, gf);
  pcio::save_plane_mesh("out.ply", result.points, descs, {120,120,120}, 180, gf);

  // Diagnostic images showing all three stages:
  pcio::GridImageConfig img;
  img.erode_iters = 1; img.dilate_iters = 1;
  img.write_density = true;   // raw heatmap
  img.write_diff    = true;   // threshold: green/red
  img.write_morph   = true;   // cyan=kept, yellow=dilated back, red=eroded away
//  pcio::save_grid_images("debug/plane", cloud.points, descs, img);
// 5/5/2026  pcio::save_grid_images("plane", cloud.points, descs, img);
  pcio::save_grid_images("plane", result.points, descs, img);
//  printf("%d images written\n", s.n_images);

  pcio::OffsetConfig off;
//  off.distances  = {0.05f, 0.10f, 0.20f};  // three shells
  off.distances  = {0.10f};  // shell
  off.both_sides = false;                     // ±distance
  off.pad_x     = 0.10f;                   // 5cm margin beyond hull bbox
  off.pad_y     = 0.10f;                   // 5cm margin beyond hull bbox
  off.pad_z     = 0.00f;                   // 5cm margin beyond hull bbox
  off.alpha      = 140;                     // face transparency
  off.grid       = gf;                      // reuse same grid filter
  off.sides   = {pcio::NEG_SIDE, pcio::POS_SIDE, pcio::NONE}; // write ... sides of each plane,here for 3 planes!!

  auto s = pcio::save_offset_planes("offsets.ply", result.points, descs, off);
  // s.n_vertices = n_planes × n_distances × (1 or 2 sides) × 4
  // s.n_faces    = same × 2
}