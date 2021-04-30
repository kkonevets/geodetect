// "Copyright 2020 Kirill Konevets"

//!
//! @file geometry.hpp
//! @brief Set of geo detection functions
//!

#ifndef INCLUDE_GEOMETRY_HPP_
#define INCLUDE_GEOMETRY_HPP_

#include <Eigen/Dense>

#include "geometry.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>

namespace x_company::geometry {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using Geometry = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::ColMajor>;
using Point = Eigen::Matrix<double, 1, 2>;
using PointXY = bg::model::d2::point_xy<double>;
using Box = bg::model::box<PointXY>;

constexpr double length_meridian = 20004274;
// 1 meter in degrees
constexpr double METER = 180. / length_meridian;

enum class Shape : char { polygon, line, circle };

//! A polygon can contain fully, intersect or not intersect with a circle.
//! If it does not intersect it is outside of a polygon.
struct Condition {
  bool intersects = false;
  bool contains = false;
  bool operator==(const Condition &rhs) const;
  friend std::ostream &operator<<(std::ostream &out, const Condition &c);
};

//! Vectorised Ramer–Douglas–Peucker algorithm
//! \param g vector of points representing a 2D geometry
//! \param epsilon throwing away threshold
//! \return simplified geometry with some points thrown away
auto rdp(const Geometry &g, Geometry::Scalar epsilon) -> Geometry;

auto rdp(const Geometry &&g, Geometry::Scalar epsilon) -> Geometry;

//!
//! Calculates adjacent row differences.
//! g:
//! 10   2
//! 5    8
//! 3    4
//!
//! diff(g):
//! -5   6
//! -2  -4
//!
//! \return Geomery object with one number of rows less
auto row_diff(const Geometry &g) -> Geometry;

//! Vector cross product of a 2D `point` by a vector of 2D points `g`.
//! \param g vector of points representin a 2D geometry
//! \return vector of components along `z` axis
auto cross(Point point, const Geometry &g) -> Eigen::VectorXd;

auto cross(Point point, const Geometry &&g) -> Eigen::VectorXd;

//! Calculates distances of every point of `g` to a line connecting
//! beginning and end of `g`
//! \param g vector of points representin a 2D geometry
//! \return vector of distances to the line
auto line_dists(const Geometry &g, const Point &begin, const Point &end)
    -> Eigen::VectorXd;

//! Calculate mean L2 distance between adjacent points in geometry
auto mean_distance(const Geometry &g) -> Geometry::value_type;

//! Calculate median L2 distance between adjacent points in geometry
auto median_distance(const Geometry &g) -> Geometry::value_type;

//! Converts wkt text to geometry, type of geometry should be known in advance
//! \param wkt string in wkt format
//! \param s geometry shape enum value
//! \returns geometry data in ColMajor format
auto parse_wkt(std::string_view wkt, Shape s) -> std::vector<double>;

//! Helper function to load points from txt file.
//! \param fname input file name
//! \return geometry from file
auto load_points_csv(const std::string &fname) -> Geometry;

//! make box centered in point with a side length equal to 2*accr
//! \param p center of a box
//! \param accr half length of a box size
//! \returns centered box
inline auto centered_box(const Point p, double accr) -> Box {
  static const double root = std::sqrt(2);
  double diag = root * accr;
  Box box(PointXY(p(0) - diag, p(1) - diag),  // left bottom corner
          PointXY(p(0) + diag, p(1) + diag)); // top right corner
  return box;
}

//! Calculates the envelope of a geometry
//! \param g geometry
//! \returns bounding box around geometry
inline auto envelope(const Geometry &g) -> Box {
  double minx = g.col(0).minCoeff();
  double maxx = g.col(0).maxCoeff();
  double miny = g.col(1).minCoeff();
  double maxy = g.col(1).maxCoeff();

  // calculate polygon bounding box
  return Box(PointXY(minx, miny), PointXY(maxx, maxy));
}

//! `pnpoly` algorithm to find if a point is in a polygon. W. Randolph Franklin
//! \param poly polygon points
//! \param point center of a circle
//! \returns point is in a polygon or not
bool pnpoly(const Geometry &poly, const Point &point);

//! Find minimum distance from a point to a geometry as a minimum
//! distances to all edges
//! \param g geometry points
//! \param point to find distance from
//! \return minimum distance to a polygon
double minimum_distance(const Geometry &g, const Point &point);

//! Detect if polygon contains a circle, if it does't detect intersection.
//! \param poly polygon points
//! \param point center of a circle
//! \param accr circle radius (accuracy)
//! \returns condition of intersection - intesects or/and contains
Condition contains(const Geometry &poly, const Point &point, double accr);

//! Detect if a circle intersects with a polygon
//! \param poly polygon points
//! \param point center of a circle
//! \param accr circle radius (accuracy)
//! \returns true if intersects, false otherwise
bool intersects(const Geometry &poly, const Point &point, double accr);

} // namespace x_company::geometry

#endif // INCLUDE_GEOMETRY_HPP_
