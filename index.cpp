#include <boost/format.hpp>
#include <fstream>
#include <memory>
#include <pqxx/pqxx>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <fmt/format.h>
#include <company/Log.hpp>

#include "index.hpp"

namespace x_company::geometry {

namespace bgi = boost::geometry::index;

geoindex::geoindex(const std::string &options, size_t threshold)
    : conn_(options), threshold_{threshold}, //
      listener_{conn_, "geometry_modify"} {
  // trigger notifications for all geometries
  initialize();
  // process triggered notifications and wait till index is filled
  //update_index();
  worker_ = std::thread([&]() {
    while (!m_finished) {
      try {
        update_index();
      }
      catch(const pqxx::failure& e){
        XK_LOGERR << "geoindex error: " << e.what();
      }
    }
  });
  XK_LOGINFO << fmt::format("geoindex: loaded {} geometries", rtree_.size());
  // start receiving notifications in a loop
}


void geoindex::stop(){
  conn_.close();
  m_finished = true;
}

geoindex::~geoindex() {
  this->stop();
  // wait for update loop
  if (worker_.joinable())
    worker_.join();
}

auto geoindex::intersect(const Point &point, double accr)
    -> std::unordered_set<std::int32_t> {
  // convert meters to degrees
  accr *= METER;

  std::unordered_set<std::int32_t> result;
  auto box = centered_box(point, accr);

  // busy wait till messages are received, decreases number of readers so that
  // write lock could be acquired without deferring. In other words, enables
  // write-preferring readers–writer lock
  while (!received_.load(std::memory_order_acquire)) {
  }
  // shared lock for reading
  std::shared_lock lock(mutex_);

  //  search in rtree
  for (auto it = rtree_.qbegin(bgi::intersects(box)); it != rtree_.qend();
       ++it) {
    // `it` is a pointer to `Value` = pair<box, Key>
    auto [shape, id] = it->second;
    auto iit = index_.find(it->second);
    if (iit == index_.end()) {
      auto fmt = boost::format("geoindex: can't find geometry "
                               "type %1% with row id %2%") %
                 static_cast<int>(shape) % id;
      throw std::logic_error(boost::str(fmt));
    }

    auto &[g, buffer, gz_id] = iit->second;

    // intersect found geometry with a circle
    bool success = false;
    if (shape == Shape::polygon) {
      success = intersects(g, point, accr);
    } else if (shape == Shape::line) {
      auto dist = minimum_distance(g, point);
      success = dist <= buffer + accr;
    } else if (shape == Shape::circle) {
      // distance between two points
      auto dist = (g - point).rowwise().norm()(0);
      success = dist <= buffer + accr;
    } else {
      throw std::logic_error("Unknown shape");
    }

    if (success)
      result.insert(gz_id);
  }

  return result;
}

auto geoindex::size() const -> size_t { return index_.size(); }

auto geoindex::find(const Key &key) const -> Index::const_iterator {
  return index_.find(key);
}

auto geoindex::cbegin() const -> Index::const_iterator {
  return index_.cbegin();
}
auto geoindex::cend() const -> Index::const_iterator { return index_.cend(); }

//! Map geometry shape string to enum
inline auto str2shape(std::string_view shape) -> Shape {
  static const std::unordered_map<std::string_view, Shape> kv{
      {"polygon", Shape::polygon},
      {"line", Shape::line},
      {"circle", Shape::circle}};
  auto it = kv.find(shape);
  if (it == kv.end())
    throw std::logic_error("Unknown shape type: " + std::string(shape));
  return it->second;
}

//! Map geometry shape to buffer name
inline auto buffer_tag(std::string_view shape_str) -> std::string {
  static const std::unordered_map<std::string_view, std::string> kv{
      {"polygon", "NULL"}, {"line", "width"}, {"circle", "radius"}};
  auto it = kv.find(shape_str);
  if (it == kv.end())
    throw std::logic_error("Unknown shape type: " + std::string(shape_str));
  return it->second;
}

inline auto make_box(const Geometry &g, Shape s, double buff) -> Box {
  if (s == Shape::circle)
    return centered_box(g, buff);
  else
    return envelope(g);
}

void geoindex::initialize() {
  // unite all tables
  std::string query = R"(
  do language plpgsql $$
    DECLARE r record;
  begin
    FOR r IN
      select id, 'gz_polygon' as tbl from geo.gz_polygon
      UNION
      select id, 'gz_line' from geo.gz_line
      UNION
      select id, 'gz_circle' from geo.gz_circle
    LOOP
      PERFORM pg_notify('geometry_modify', format('(%s,%s,INSERT)', r.id, r.tbl));
    END LOOP;
  end$$;
  )";

  {
    // send notifications to receiver
    pqxx::work tx{conn_};
    tx.exec0(query.data());
    tx.commit();
  }
}

void geoindex::update_index() {
  conn_.await_notification();
  // we can batch process with a threshold > 1 if we wish
  if (listener_.size() < threshold_)
    return;

  // consume current messages:
  // DELETE: just remove
  // UPDATE: remove then insert (Ok for rtree, not optimal for index)
  // INSERT: just insert
  while (!listener_.empty()) {
    auto msg = listener_.pop();
    if (msg.op == "DELETE" || msg.op == "UPDATE") {
      to_remove_.push_back(msg);
    }
    if (msg.op == "INSERT" || msg.op == "UPDATE") {
      auto it = to_query_.find(msg.shape);
      if (it == to_query_.end())
        to_query_.insert({msg.shape, {msg.id}});
      else
        it->second.push_back(msg.id);
    }
  }

  // this function is the slowest
  auto result = load_triggered_data(to_query_);

  // notify reader threads that we want to write
  received_.store(false, std::memory_order_release);
  {
    // unique lock for writing
    std::unique_lock lock(mutex_);

    // remove
    for (auto &msg : to_remove_) {
      auto s = str2shape(msg.shape);
      auto key = std::make_pair(s, msg.id);
      auto it = index_.find(key);
      if (it != index_.end()) {
        auto &[g, buff, _] = it->second;
        rtree_.remove(std::make_pair(make_box(g, s, buff), key));
        index_.erase(it);
      }
    }

    // insert
    for (const auto &row : result) {
      auto type = std::string_view(row["shape"].c_str(), row["shape"].size());
      auto s = str2shape(type);
      auto wkt = std::string_view(row["wkt"].c_str(), row["wkt"].size());
      auto data = parse_wkt(wkt, s);
      auto g = Eigen::Map<Geometry>(data.data(), data.size() / 2, 2);
      auto id = row["id"].as<std::int32_t>();
      auto gz_id = row["gz_id"].as<std::int32_t>();
      auto buff = row["buffer"].as<double>(0) * METER;
      auto key = std::make_pair(s, id);
      index_.insert(std::make_pair(key, std::make_tuple(g, buff, gz_id)));
      rtree_.insert(std::make_pair(make_box(g, s, buff), key));
    }
  }
  // notify reader threads that we finished writing
  received_.store(true, std::memory_order_release);

  // cleanup
  to_remove_.clear();
  to_query_.clear();
}

auto geoindex::load_triggered_data(query_params &params) -> pqxx::result {
  constexpr std::string_view pattern =
      "select id, gz_id, ST_AsText(%1%) as wkt, %2% as buffer, '%1%' as shape "
      "from geo.gz_%1% "
      "where id in (%3%)\n";
  std::string query;

  // unite all tables (shapes) in one query
  for (query_params::iterator it = params.begin(); it != params.end(); ++it) {
    // clear buffer
    ss_.str(std::string());

    // convert vector of numbers to a comma separated string
    for (size_t i = 0; i < it->second.size(); ++i) {
      if (i != 0)
        ss_ << ",";
      ss_ << it->second[i];
    }
    if (!query.empty())
      query.append("UNION\n");

    auto btag = buffer_tag(it->first);
    auto part = boost::format(pattern.data()) % it->first % btag % ss_.str();
    query.append(part.str());
  }
  query.append(";");

  // execute the query
  pqxx::result result;
  pqxx::nontransaction work{conn_};
  result = work.exec(query);
  return result;
}

} // namespace  x_company::geometry
