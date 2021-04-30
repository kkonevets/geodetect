#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <fstream>
#include <iostream>
#include <pqxx/pqxx>
#include <string>

namespace bg = boost::geometry;
using point_type = boost::geometry::model::d2::point_xy<double>;

template <class Model> void download_shapes() {
  std::string shape_type;
  if constexpr (std::is_same_v<Model, bg::model::polygon<point_type>>)
    shape_type = "polygon";
  else if constexpr (std::is_same_v<Model, bg::model::linestring<point_type>>)
    shape_type = "line";
  else
    std::logic_error("not implemented");

  std::string sql_query = boost::str(
      boost::format("select id, ST_AsText(%1%) as %1% from geo.gz_%1%;") %
      shape_type);

  boost::filesystem::path opath("/home/guyos/Documents/data/");
  opath /= shape_type;

  pqxx::connection connection("user=guyos dbname=company_devices");
  pqxx::nontransaction work{connection};
  pqxx::result result;
  result = work.exec(sql_query.data());

  Model g;
  for (const auto &row : result) {
    auto leaf = std::string(row["id"].c_str()) + ".txt";
    std::ofstream fout((opath / leaf).string());
    fout.precision(9);
    bg::read_wkt(row[shape_type].c_str(), g);

    bg::for_each_point(g, [&fout](point_type &point) {
      fout << bg::get<1>(point) << " " << bg::get<0>(point) << std::endl;
    });

    fout.close();
  }
}

int main() {
  download_shapes<bg::model::polygon<point_type>>();
  download_shapes<bg::model::linestring<point_type>>();
}
