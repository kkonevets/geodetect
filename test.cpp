#define BOOST_TEST_MODULE geometry

#include <Eigen/Dense>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/test/unit_test.hpp>

#include "geometry.hpp"
#include "index.hpp"

using namespace std::chrono_literals;
namespace utf = boost::unit_test;
namespace fs = boost::filesystem;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
namespace geometry = x_company::geometry;

using PointXY = bg::model::d2::point_xy<double>;
using Box = bg::model::box<PointXY>;
using Polygon = bg::model::polygon<PointXY>;
using Line = bg::model::linestring<PointXY>;
using Value = std::pair<Box, size_t>;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using geometry::Condition;
using geometry::geoindex;
using geometry::Geometry;
using geometry::Point;
using geometry::Shape;

// precision when comparing double numbers
constexpr double PREC = 1E-10;
constexpr auto LAG = 500ms; // wait for notification

// generate random number in interval [from, to]
template <typename Numeric, typename Generator = std::mt19937>
Numeric random(Numeric from, Numeric to) {
  thread_local static Generator gen(std::random_device{}());
  using dist_type =
      typename std::conditional<std::is_integral<Numeric>::value,
                                std::uniform_int_distribution<Numeric>,
                                std::uniform_real_distribution<Numeric>>::type;
  thread_local static dist_type dist;
  return dist(gen, typename dist_type::param_type{from, to});
}

inline auto commit(pqxx::connection &conn, const std::string &query)
    -> pqxx::result {
  pqxx::work tx{conn};
  auto result = tx.exec(query);
  tx.commit();
  return result;
}

BOOST_AUTO_TEST_CASE(geometry_modify, *utf::disabled()) {

#define IPOLY "POLYGON((0 0,1 0,1 1,0 1,0 0))"
#define UPOLY "POLYGON((0 0,2 0,2 2,0 2,0 0))"
#define ILINE "LINESTRING(0 0,2 0,1 1,0 1)"
#define ULINE "LINESTRING(0 0,3 0,1 1,0 1)"
#define IPOINT "POINT(0 0)"
#define UPOINT "POINT(4 3)"

  std::string insert_polygon = //
      "insert into geo.gz_polygon(gz_id, polygon) values "
      "(1, ST_AsBinary(ST_GeomFromText('" IPOLY "'))) "
      "RETURNING id;";
  std::string update_polygon = //
      "update geo.gz_polygon set "
      "polygon=ST_AsBinary(ST_GeomFromText('" UPOLY "')) where id=%1%;";
  std::string delete_polygon = //
      "delete from geo.gz_polygon where id=%1%;";

  std::string insert_line = //
      "insert into geo.gz_line(gz_id, line, width) values "
      "(1, ST_AsBinary(ST_GeomFromText('" ILINE "')), 2) "
      "RETURNING id;";
  std::string update_line = //
      "update geo.gz_line set width=3, "
      "line=ST_AsBinary(ST_GeomFromText('" ULINE "'))"
      " where id=%1%;";
  std::string delete_line = //
      "delete from geo.gz_line where id=%1%;";

  std::string insert_circle = //
      "insert into geo.gz_circle(gz_id, circle, radius) values "
      "(1, ST_AsBinary(ST_GeomFromText('" IPOINT "')), 5) "
      "RETURNING id;";
  std::string update_circle = //
      "update geo.gz_circle set radius=7, "
      "circle=ST_AsBinary(ST_GeomFromText('" UPOINT "')) where id=%1%;";
  std::string delete_circle = //
      "delete from geo.gz_circle where id=%1%;";

  std::string options{"user=guyos dbname=company_devices"};
  auto index = geoindex(options);
  pqxx::connection conn(options);

  auto test_shape = [&conn, &index](std::string &insert_q,
                                    std::string &update_q,
                                    std::string &delete_q, std::string iwkt,
                                    std::string uwkt, Shape s,
                                    double insert_buff, double update_buff) {
    struct Fields {
      std::vector<double> data;
      double buffer;
      int32_t gz_id;
    };

    auto check_fields = [](Fields &fs,
                           const geoindex::Index::mapped_type &val) {
      auto &[g, buffer, gz_id] = val;
      auto fs_g = Eigen::Map<Geometry>(fs.data.data(), fs.data.size() / 2, 2);
      BOOST_CHECK_EQUAL(g, fs_g);
      BOOST_CHECK_EQUAL(buffer, fs.buffer * geometry::METER);
      BOOST_CHECK_EQUAL(gz_id, fs.gz_id);
    };

    auto test_insert = [&](Shape s, std::string &query,
                           Fields &fs) -> std::int32_t {
      size_t prev_size = index.size();
      pqxx::result result = commit(conn, query);
      auto id = result[0]["id"].as<std::int32_t>();

      std::this_thread::sleep_for(LAG);
      BOOST_CHECK_EQUAL(index.size(), prev_size + 1);

      auto it = index.find(std::make_pair(s, id));
      BOOST_REQUIRE(it != index.cend());

      check_fields(fs, it->second);
      return id;
    };

    auto test_update = [&](Shape s, std::string &query, std::int32_t id,
                           Fields &fs) {
      size_t prev_size = index.size();
      auto key = std::make_pair(s, id);
      auto prev_val = index.find(key)->second;
      commit(conn, query);

      std::this_thread::sleep_for(LAG);
      BOOST_CHECK_EQUAL(index.size(), prev_size);

      auto val = index.find(key)->second;
      BOOST_REQUIRE(prev_val != val);

      check_fields(fs, val);
    };

    auto test_delete = [&](Shape s, std::string &query, std::int32_t id) {
      size_t prev_size = index.size();
      auto key = std::make_pair(s, id);
      commit(conn, query);

      std::this_thread::sleep_for(LAG);
      BOOST_CHECK_EQUAL(index.size(), prev_size - 1);

      auto it = index.find(key);
      BOOST_REQUIRE(it == index.cend());
    };

    Fields fls{parse_wkt(iwkt, s), insert_buff, 1};
    std::int32_t id = test_insert(s, insert_q, fls);
    fls = {parse_wkt(uwkt, s), update_buff, 1};
    std::string query = boost::str(boost::format(update_q) % id);
    test_update(s, query, id, fls);
    query = boost::str(boost::format(delete_q) % id);
    test_delete(s, query, id);
  };

  test_shape(insert_polygon, update_polygon, delete_polygon, std::string(IPOLY),
             std::string(UPOLY), Shape::polygon, 0, 0);
  test_shape(insert_line, update_line, delete_line, std::string(ILINE),
             std::string(ULINE), Shape::line, 2, 3);
  test_shape(insert_circle, update_circle, delete_circle, std::string(IPOINT),
             std::string(UPOINT), Shape::circle, 5, 7);

  // test UPDATE and DELETE in a transaction
  auto size = index.size();
  pqxx::result result = commit(conn, insert_line);
  auto id = result[0]["id"].as<std::int32_t>();

  std::string query = "update geo.gz_line set width=3 where id=%1%;"
                      "delete from geo.gz_line where id=%1%;";
  auto res = commit(conn, boost::str(boost::format(query) % id));

  std::this_thread::sleep_for(LAG);
  BOOST_REQUIRE(size == index.size());

  std::cout << "finished testing "
            << boost::unit_test::framework::current_test_case().p_name
            << std::endl;
  exit(0);
}

BOOST_AUTO_TEST_CASE(geometry_read, *utf::disabled()) {
  std::string options{"user=guyos dbname=company_devices"};
  auto index = geoindex(options);

  //  =============================POLYGON============================================

  Point p(61.174959, 80.690492);         // lon, lat
  auto gz_ids = index.intersect(p, 100); // in meters
  BOOST_REQUIRE(std::find(gz_ids.begin(), gz_ids.end(), 96) != gz_ids.end());

  p = {62.304553, 80.560222};       // lon, lat
  gz_ids = index.intersect(p, 500); // in meters
  BOOST_REQUIRE(std::find(gz_ids.begin(), gz_ids.end(), 96) != gz_ids.end());

  p = {62.346812, 81.049842};       // lon, lat
  gz_ids = index.intersect(p, 100); // in meters
  BOOST_REQUIRE(std::find(gz_ids.begin(), gz_ids.end(), 96) == gz_ids.end());

  //  ===============================LINE============================================

  p = {41.174837, 54.146784};       // lon, lat
  gz_ids = index.intersect(p, 100); // in meters
  BOOST_REQUIRE(std::find(gz_ids.begin(), gz_ids.end(), 1026) != gz_ids.end());

  p = {41.187213, 54.163396};       // lon, lat
  gz_ids = index.intersect(p, 100); // in meters
  BOOST_REQUIRE(std::find(gz_ids.begin(), gz_ids.end(), 1026) == gz_ids.end());

  //  =============================CIRCLE============================================

  p = {30.434060, 50.462175};       // lon, lat
  gz_ids = index.intersect(p, 100); // in meters
  BOOST_REQUIRE(std::find(gz_ids.begin(), gz_ids.end(), 1052) != gz_ids.end());

  // this does not work because real distance from (30.433608, 50.462526) is 230
  // meters, but our formula gives 255 meters
  p = {30.434976, 50.460683}; // lon, lat

  p = {30.435018, 50.461281};       // lon, lat
  gz_ids = index.intersect(p, 100); // in meters
  BOOST_REQUIRE(std::find(gz_ids.begin(), gz_ids.end(), 1052) != gz_ids.end());

  p = {30.441640, 50.458475};       // lon, lat
  gz_ids = index.intersect(p, 100); // in meters
  BOOST_REQUIRE(std::find(gz_ids.begin(), gz_ids.end(), 1052) == gz_ids.end());

  std::cout << "finished testing "
            << boost::unit_test::framework::current_test_case().p_name
            << std::endl;
  exit(0);
}

BOOST_AUTO_TEST_CASE(geometry_par, *utf::disabled()) {
  std::string options{"user=guyos dbname=company_devices"};
  auto index = geoindex(options);
  pqxx::connection conn(options);

  Eigen::initParallel();

  // generate random points from Russia
  std::vector<Point> points;
  for (size_t i = 0; i < 100'000; i++) {
    points.emplace_back(random<double>(25, 169),
                        random<double>(41, 71)); // lon, lat
  }

  auto do_work = [&index, &points](size_t from, size_t to) {
    for (size_t i = from; i < to; i++) {
      index.intersect(points[i], 200); // 200 meters
    }
  };

  // create threads
  std::vector<std::thread> threads;
  size_t nthreads = 4;
  size_t chunk_size = points.size() / nthreads;
  size_t from = 0;
  auto size = index.size();
  int num = 10;
  auto start = std::chrono::high_resolution_clock::now();

  for (size_t i = 0; i < nthreads; i++) {
    size_t to = from + chunk_size;
    threads.emplace_back(do_work, from, to);
    from = to;
  }

  std::string insert_q = //
      "insert into geo.gz_polygon(gz_id, polygon) values "
      "(1, ST_AsBinary(ST_GeomFromText('POLYGON((1 1,2 1,2 2,1 1))'))) "
      "RETURNING id;";
  std::string delete_q = "delete from geo.gz_polygon where id=%1%;";

  // test write while parallel read
  for (int i = 0; i < num; i++) {
    pqxx::result result = commit(conn, insert_q);
    std::this_thread::sleep_for(10ms);
    BOOST_CHECK_EQUAL(index.size(), size + 1);

    auto id = result[0]["id"].as<std::int32_t>();
    auto fmt = boost::format(delete_q) % id;
    commit(conn, fmt.str());
    std::this_thread::sleep_for(10ms);
    BOOST_CHECK_EQUAL(index.size(), size);
  }

  for (auto &th : threads) {
    th.join();
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds> //
                  (end - start).count();
  std::cout << "duration: " << duration << " sec" << std::endl;

  std::cout << "finished testing "
            << boost::unit_test::framework::current_test_case().p_name
            << std::endl;
  exit(0);
}

BOOST_AUTO_TEST_CASE(geometry_speed, *utf::disabled()) {
  fs::path p(__FILE__);
  p = p.remove_leaf() / "../../data/";

  // read all polygons to memory and save their file names for info
  std::vector<Geometry> geoms;
  std::vector<std::string> names;
  // every polygon is in it's own file
  std::vector<std::string> dirs{"polygon"};
  for (auto &dir_name : dirs) {
    auto currp = p / dir_name;
    std::cout << currp << std::endl;
    fs::directory_iterator end_itr;
    // cycle through the directory
    for (fs::directory_iterator itr(currp); itr != end_itr; ++itr) {
      std::string fname = itr->path().string();
      geoms.push_back(geometry::load_points_csv(fname));
      names.push_back(std::move(fname));
    }
  }

  // create spacial index (rtree) and keep it in memory
  // Algorithm:
  // 1) get polygons bounding boxes and save them to index
  // 2) get bounding box of a query circle and search it in index
  // 3) intersect matched polygons on step 2) with a circle

  // STAGE 1)
  bgi::rtree<Value, bgi::rstar<16>> rtree;
  for (size_t i = 0; i < geoms.size(); ++i) {
    auto &g = geoms[i];
    // calculate polygon bounding box
    auto box = geometry::envelope(g);
    // insert polygon box with it's index in `geoms`
    rtree.insert(std::make_pair(box, i));
  }

  // generate 100k random points from Russia
  std::vector<Point> points;
  for (size_t i = 0; i < 100'000; i++) {
    points.emplace_back(random<double>(41, 71),
                        random<double>(25, 169)); // lat, lon
  }

  double accr = 200 * geometry::METER;

  auto start = std::chrono::high_resolution_clock::now();
  size_t i = 0;
  for (auto point : points) {
    // STAGE 2) make query box
    auto box = geometry::centered_box(point, accr);
    size_t inter_count = 0;
    // std::cout << point << std::endl;
    // STAGE 2) search in rtree
    for (auto it = rtree.qbegin(bgi::intersects(box)); it != rtree.qend();
         ++it) {
      // `it` is a pointer to `Value` = pair<polygon box, polygon index>
      // STAGE 3)
      bool b = geometry::intersects(geoms[it->second], point, accr);
      if (b) {
        // std::cout << names[it->second] << " ";
      }
      inter_count++;
    }
    // std::cout << std::endl;
    if (i % 1000 == 0)
      std::cout << i << " " << inter_count << std::endl;
    i++;
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::seconds> //
                  (end - start).count();
  std::cout << "duration: " << duration << " sec" << std::endl;
}

BOOST_AUTO_TEST_CASE(geometry_rdp_speed, *utf::disabled()) {
  fs::path p(__FILE__);
  p = p.remove_leaf().parent_path() / "data/polygon/3813.txt";
  Geometry g = geometry::load_points_csv(p.string());
  auto epsilon = geometry::median_distance(g);

  int num_iter = 7;
  auto t1 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < num_iter; i++) {
    auto simplified = geometry::rdp(g, epsilon);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

  std::cout << duration / num_iter << std::endl;
}

void test_file(const std::string &fname) {
  Geometry g = geometry::load_points_csv(fname + ".txt");
  Geometry result = geometry::load_points_csv(fname + "_result.txt");

  auto epsilon = geometry::mean_distance(g);
  auto simplified = geometry::rdp(g, epsilon);
  BOOST_REQUIRE(simplified.isApprox(result, PREC));
}

BOOST_AUTO_TEST_CASE(geometry_rdp) {
  fs::path p(__FILE__);
  p = p.remove_leaf() / "data";

  test_file((p / "1").string());
  test_file((p / "1202").string());
  test_file((p / "1742").string());
  test_file((p / "2650").string());
  test_file((p / "3736").string());
}

BOOST_AUTO_TEST_CASE(geometry_cross) {
  Geometry g(4, 2);
  g << 3, -1, //
      2.5, 5, //
      7, 9.1, //
      1, 2;
  Eigen::VectorXd result(4);
  result << -4.4, -0.75, -4.55, -0.3;

  Point point(1, 2);
  point << 0.5, 1.3;

  auto cross = geometry::cross(point, g);
  BOOST_REQUIRE(cross.isApprox(result, PREC));
}

BOOST_AUTO_TEST_CASE(geometry_line_dists) {
  Geometry g(4, 2);
  g << 3, -1, //
      2.5, 5, //
      7, 9.1, //
      1, 2;

  Eigen::VectorXd result(4);
  result << 0., 2.91217603, 8.93067316, 0.;

  Point begin = g.row(0), end = g.row(g.rows() - 1);
  auto dists = geometry::line_dists(g, begin, end);
  BOOST_REQUIRE(dists.isApprox(result, PREC));
}

BOOST_AUTO_TEST_CASE(geometry_pnpoly) {
  Geometry poly(5, 2);
  poly << 2, 2, //
      7, 8,     //
      5, 3,     //
      7, 1,     //
      2, 2;

  Geometry points(6, 2);
  points << 1, 2, //
      3, 6,       //
      6, 4,       //
      4, 3,       //
      3, -1,      //
      2, 2;

  std::vector<bool> answers{false, false, false, true, false, true};

  for (int i = 0; i < points.rows(); i++) {
    auto point = points.row(i);
    auto result = geometry::pnpoly(poly, point);
    BOOST_CHECK_EQUAL(result, answers[i]);
  }
}

BOOST_AUTO_TEST_CASE(geometry_min_dist) {
  Geometry g(4, 2); // a line
  g << 2, 2,        //
      6, 6,         //
      6, 2,         //
      6, 2;         // add zero edge

  Geometry points(5, 2);
  points << 6, 6, //
      4, 3,       //
      3, 5,       //
      8, 4,       //
      7, 1;

  std::vector<double> answers{0, std::sqrt(2) / 2, std::sqrt(2), 2,
                              std::sqrt(2)};

  for (int i = 0; i < points.rows(); i++) {
    auto point = points.row(i);
    auto dist = geometry::minimum_distance(g, point);
    BOOST_CHECK(std::abs(dist - answers[i]) < PREC);
  }
}

BOOST_AUTO_TEST_CASE(geometry_intersect) {
  Geometry poly(9, 2);
  poly << 2, 2.5, //
      5, 4,       //
      6.5, 7,     //
      8, 4,       //
      11, 2.5,    //
      8, 1,       //
      6.5, -2,    //
      5, 1,       //
      2, 2.5;

  Geometry points(9, 2);
  points << 2.5, 4.5, //
      2.5, 4.5,       //
      5, 1,           //
      5, 1,           //
      6.5, 2.5,       //
      6.5, 2.5,       //
      6.5, 2.5,       //
      0, 0,           //
      0, 0;

  std::vector<double> accrs{1,   1.6, 1,                         //
                            100, 1.5, 1.5 * std::sqrt(2) - PREC, //
                            3,   2,   4};
  std::vector<Condition> answers{
      {false, false}, {true, false},  {true, false}, //
      {true, false},  {true, true},   {true, true},  //
      {true, false},  {false, false}, {true, false}};

  for (int i = 0; i < points.rows(); i++) {
    auto point = points.row(i);
    auto result = geometry::contains(poly, point, accrs[i]);
    BOOST_CHECK(result == answers[i]);
  }
}
