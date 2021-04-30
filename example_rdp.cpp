#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

#include "geometry.hpp"

namespace geometry = x_company::geometry;

void rdp_file(std::string fname) {
  auto g = geometry::load_points_csv(fname);
  auto epsilon = geometry::median_distance(g);
  auto result = geometry::rdp(g, epsilon);

  std::ofstream fout("./rdp_out.txt");
  fout.precision(9);

  for (int i = 0; i < result.rows(); i++) {
    const auto &row = result.row(i);
    fout << row(0) << " " << row(1) << std::endl;
  }
  fout.close();
}

int main() {
  // WKT format
  std::string text = "LINESTRING(36 11,40 43,22 43,15 20,30 16)";
  auto data = geometry::parse_wkt(text, geometry::Shape::line);
  // now data is {36 40 22 15 30 11 43 43 20 16}

  // constructing Geometry: take a view of vector, NO COPY
  auto g = Eigen::Map<geometry::Geometry>(data.data(), data.size() / 2, 2);
  std::cout << "Before: " << std::endl;
  std::cout << g << std::endl << std::endl;

  // calculate median distance between adjacent points
  auto epsilon = geometry::median_distance(g);
  // apply rdp
  auto simplified = geometry::rdp(g, epsilon);
  std::cout << "After: " << std::endl;
  std::cout << simplified << std::endl;

  return 0;
}

/*

Output:

Before:
36 11
40 43
22 43
15 20
30 16

After:
36 11
40 43
30 16

*/
