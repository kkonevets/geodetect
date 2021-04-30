// "Copyright 2020 Kirill Konevets"

//!
//! @file geometry.cpp
//! @brief Math functions for geodetection
//!

#include <Eigen/Dense>
#include <algorithm>
#include <boost/spirit/include/qi.hpp>
#include <fstream>
#include <iostream>
#include <string_view>
#include <vector>

#include "geometry.hpp"

using Eigen::VectorXd;

namespace x_company::geometry {

bool Condition::operator==(const Condition &rhs) const {
  return intersects == rhs.intersects && contains == rhs.contains;
}

std::ostream &operator<<(std::ostream &out, const Condition &c) {
  out << "(" << c.intersects << ", " << c.contains << ")";
  return out;
}

auto rdp(const Geometry &g, Geometry::Scalar epsilon) -> Geometry {
  if (g.rows() <= 1)
    return g;

  Point begin = g.row(0), end = g.row(g.rows() - 1);
  auto dists = line_dists(g, begin, end);

  Geometry::Index index = 0;
  Geometry::Scalar dmax = dists.maxCoeff(&index);

  if (dmax > epsilon) {
    auto sub1 = rdp(g.topRows(index + 1), epsilon);
    auto sub2 = rdp(g.bottomRows(g.rows() - index), epsilon);

    if (sub1.rows() == 0) {
      return sub2;
    } else {
      Geometry result(sub1.rows() - 1 + sub2.rows(), sub1.cols());
      result << sub1.topRows(sub1.rows() - 1), sub2;
      return result;
    }
  } else {
    Eigen::Matrix2d result;
    result << begin, end;
    return result;
  }
}

auto rdp(const Geometry &&g, Geometry::Scalar epsilon) -> Geometry {
  return rdp(g, epsilon);
}

auto cross(Point point, const Geometry &g) -> VectorXd {
  // Vector product of two points `a = [a1, a2]` and `b = [b1, b2]` is:
  // `a x b = a1*b2 - a2*b1`, which can be done by negating `b1` and then
  // reversing `b`: `reverse(b) = [b2, -b1]`, finishing with a dot product

  point(0, 1) *= -1;
  return g * point.transpose().reverse();
}

auto cross(Point point, const Geometry &&g) -> VectorXd {
  return cross(std::move(point), g);
}

auto line_dists(const Geometry &g, const Point &begin, const Point &end)
    -> VectorXd {
  // if geometry is a closed path (e.g. polygon) then return distances to the
  // point
  if (begin == end) {
    return (g.rowwise() - begin).rowwise().norm();
  }

  // get vector from begin to the end
  auto point = end - begin;
  // get cross product of the vector with vector from begin to each point from
  // `g`, divide the product by the vector norm making it a unit vector
  // cross(a,b) = |a|*|b|*sin(alpha) = |a|*1*sin(alpha) = distance to the line
  auto product = cross(point, g.rowwise() - begin);
  Point::Scalar norm = point.rowwise().norm()(0, 0);
  return product.cwiseAbs() / norm;
}

auto row_diff(const Geometry &g) -> Geometry {
  Geometry g1 = g.block(0, 0, g.rows() - 1, g.cols());
  Geometry g2 = g.block(1, 0, g.rows() - 1, g.cols());
  return g2 - g1;
}

auto parse_wkt(std::string_view wkt, Shape s) -> std::vector<double> {
  namespace qi = boost::spirit::qi;

  std::vector<double> data;
  auto first = wkt.begin();
  auto last = wkt.end();

#define DBL +qi::double_
#define SKIP qi::char_(", ")

  bool ok = false;
  if (s == Shape::polygon) {
    ok = qi::phrase_parse(first, last, "POLYGON((" >> DBL >> "))", SKIP, data);
  } else if (s == Shape::line) {
    ok = qi::phrase_parse(first, last, "LINESTRING(" >> DBL >> ")", SKIP, data);
  } else if (s == Shape::circle) {
    ok = qi::phrase_parse(first, last, "POINT(" >> DBL >> ")", SKIP, data);
  }

  if (!ok || first != last || data.size() % 2) {
    throw std::logic_error("Parsing failed: " + std::string(wkt));
  }

  // WKT data is RowMajor, but we need to make it ColMajor
  std::stable_partition(data.begin(), data.end(), [&data](double &a) {
    return ((&a - &data[0]) % 2) == 0;
  });
  return data;
}

auto load_points_csv(const std::string &fname) -> Geometry {
  std::ifstream fin(fname);
  if (!fin.is_open())
    throw " could not open file";

  std::vector<double> vin;
  double num = 0;
  while (fin >> num) {
    vin.push_back(num);
  }
  fin.close();

  // WKT data is RowMajor, but we need to make it ColMajor
  std::stable_partition(vin.begin(), vin.end(), [&vin](double const &a) {
    return ((&a - &vin[0]) % 2) == 0;
  });

  return Eigen::Map<Geometry>(vin.data(), vin.size() / 2, 2);
}

auto mean_distance(const Geometry &g) -> Geometry::value_type {
  return row_diff(g).rowwise().norm().mean();
}

auto median_distance(const Geometry &g) -> Geometry::value_type {
  Eigen::VectorXd dists = row_diff(g).rowwise().squaredNorm();
  std::vector<double> v(dists.data(), dists.data() + dists.size());
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return std::sqrt(v[v.size() / 2]); // median value
}

bool pnpoly(const Geometry &poly, const Point &point) {
  // get all points except the last point
  const auto from = poly.topRows(poly.rows() - 1); // j
  // get all points except the first one
  const auto to = poly.bottomRows(poly.rows() - 1); // i
  // vector from->to is a vector of directed edges

  // get x and y coordinate for references
  const auto vx = from.col(0).array();
  const auto vy = from.col(1).array();
  const auto wx = to.col(0).array();
  const auto wy = to.col(1).array();

  // 1) check if the point's Y coordinate is within the edge's Y-range
  // 2) check if the point is in the half-plane to the left of the extended edge
  // 3) count how many times 1) and 2) happen together
  // 4) if 3) is an odd number - point is in a polygon, otherwise outside
  auto temp = (((wy > point(1)) != (vy > point(1))) &&
               (point(0) < (vx - wx) * (point(1) - wy) / (vy - wy) + wx))
                  .count();
  return temp % 2;
}

double minimum_distance(const Geometry &g, const Point &point) {
  // Find minimum distance from a point to a polygon as a minimum
  // distances to all edges

  // get all points except the last point
  const auto from = g.topRows(g.rows() - 1);
  // get all points except the first one
  const auto to = g.bottomRows(g.rows() - 1);
  // vector from->to is a vector of directed edges

  auto to_from = to - from;
  // i.e. |to-from|^2 -  avoid a sqrt
  auto l2 = to_from.rowwise().squaredNorm();

  // Consider the line extending the segment, parameterized as
  // from + t (to - from). We find projection of point onto the line. It falls
  // where t =  [(point-from) . (to-from)] / |to-from|^2
  auto point_from = (-from).rowwise() + point;
  Eigen::ArrayXd t =
      point_from.cwiseProduct(to_from).rowwise().sum().array() / l2.array();

  // We clamp t from [0,1] to handle points outside the segment [from, to]
  t = t.min(1).max(0);

  // Projection falls on the segment
  auto proj = from.array() + to_from.array().colwise() * t;

  // point - proj vector
  auto diff = (-proj).matrix().rowwise() + point;
  // minumum distance from point to projection
  return std::sqrt(diff.rowwise().squaredNorm().minCoeff());
}

Condition contains(const Geometry &poly, const Point &point, double accr) {
  // check if point is in polygon
  bool isin = pnpoly(poly, point);

  // find minimum distance to a polygon
  auto min_dist = minimum_distance(poly, point);

  Condition cond;
  if (isin) {
    // if point is in polygon we have intersection
    cond.intersects = true;
    if (min_dist >= accr)
      cond.contains = true;
  } else {
    if (min_dist <= accr)
      cond.intersects = true;
  }

  return cond;
}

bool intersects(const Geometry &poly, const Point &point, double accr) {
  // check if point is in polygon
  bool isin = pnpoly(poly, point);
  if (isin) {
    // if point is in polygon we have intersection
    return true;
  } else {
    // find minimum distance to a polygon
    auto min_dist = minimum_distance(poly, point);
    if (min_dist <= accr)
      return true;
  }
  return false;
}

} // namespace x_company::geometry
