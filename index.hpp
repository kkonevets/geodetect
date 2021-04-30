// "Copyright 2020 Kirill Konevets"

//!
//! @file geoindex.hpp
//! @brief Set of geo detection functions
//!

#ifndef INCLUDE_GEOINDEX_HPP_
#define INCLUDE_GEOINDEX_HPP_

#include <atomic>
#include <map>
#include <pqxx/pqxx>
#include <shared_mutex>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "geometry.hpp"
#include "receiver.hpp"

namespace x_company::geometry {

//! In memory indexer that listenes Postges notifications in a separate thread,
//! loads modified geometries from Postges and saves them to index, extracts
//! bounding boxes of geometries and saves to rtree. Should be used to intersect
//! points with geometries
class geoindex {
public:
  // <geometry shape, row id>
  using Key = std::pair<Shape, std::int32_t>;
  // value of type  <geometry, buffer, gz_id>
  using Index = std::map<Key, std::tuple<Geometry, double, std::int32_t>>;

  //! Fills the index and starts new thread to receive notifications
  //! \param threshold minimum number of messages in a listener queue to start
  //! processing
  explicit geoindex(const std::string &options, size_t threshold = 1);
  geoindex(const geoindex &) = delete;
  geoindex(geoindex &&) = delete;
  ~geoindex();
  //! Find all geozones that intersect with a circle
  //! \param point center of a circle (longitude, latitude)
  //! \param accr radius of a circle in meters
  //! \returns set of geozone ids that intersect with a circle
  auto intersect(const Point &point, double accr)
      -> std::unordered_set<std::int32_t>;
  //! Searches the container for an element with a key equivalent to key
  //! and returns an iterator to it if found, otherwise it returns an
  //! iterator to index::cend
  //! \param key is a pair <geometry shape, row id>
  //! \returns const iterator
  auto find(const Key &key) const -> Index::const_iterator;
  //! Returns an const_iterator pointing to the first element in the index
  auto cbegin() const -> Index::const_iterator;
  //! Returns a const_iterator pointing to the past-the-end element in the index
  auto cend() const -> Index::const_iterator;
  //! Get number of geometries in index
  auto size() const -> size_t;

  void stop();

private:
  bool m_finished = false;

  // "geometry shape" -> "message.id"s lookup
  using query_params =
      std::unordered_map<std::string, std::vector<std::int32_t>>;
  // rtree value
  using Value = std::pair<Box, Key>;
  // mutex for shared reading and unique writing
  std::shared_mutex mutex_;
  // indicates messages were received;
  // should use atomic otherwise it would not be synchronised between readers
  // and writer threads
  std::atomic<bool> received_{true};

  pqxx::connection conn_;
  // minimum number of messages in a listener queue to start processing
  size_t threshold_;
  // notification receiver
  trigger_listener listener_;
  // the index, maps Postgres row to <geometry, buffer, gz_id>
  Index index_;
  // search engine that intersects geometries bounding boxes
  bgi::rtree<Value, bgi::rstar<16>> rtree_;
  // thread that listenes to Postgres notifications
  std::thread worker_;

  // helper buffers:
  // stream to convert vector to a string
  std::stringstream ss_;
  std::vector<message> to_remove_;
  query_params to_query_;

  //! Triggers all geometries to fill the index
  void initialize();
  //! Await new notifications and update the index.
  //! When new notification arrives queries database to get modified data
  //! (INSERT, UPDATE, DELETE)
  void update_index();
  //! Builds sql query and executes it.
  //! \param params ids grouped by geometry type (shape) for sql UNION
  //! \returns sql query result
  auto load_triggered_data(query_params &params) -> pqxx::result;
};


void fix_nested_geozones(std::unordered_set<std::int32_t>& geozones);

} // namespace x_company::geometry

#endif // INCLUDE_GEOINDEX_HPP_
