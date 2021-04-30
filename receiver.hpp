// "Copyright 2020 Kirill Konevets"

//!
//! @file receiver.hpp
//! @brief Set of geo detection functions
//!

#ifndef INCLUDE_RECEIVER_HPP_
#define INCLUDE_RECEIVER_HPP_

#include <pqxx/notification>
#include <pqxx/pqxx>
#include <queue>
#include <string>

namespace x_company::geometry {

//! Parsed Postges notification message
struct message {
  std::int32_t id;   // row id
  std::string shape; // TG_TABLE_NAME
  std::string op;    // TG_OP - operation
};

//! Listenes to Postgres notifications on a channel
class trigger_listener : public pqxx::notification_receiver {
  // keeps queue of current messages
  std::queue<message> msgs_;

public:
  //! \param conn postgres connection
  //! \param channel_name channel name
  explicit trigger_listener(pqxx::connection &conn,
                            const std::string &channel_name)
      : pqxx::notification_receiver(conn, channel_name) {}
  //! Callback on message receiving
  //! \param payload message string from Postges
  void operator()(std::string const &payload, int) override;
  //! Get number of current messages in a queue
  auto size() const -> size_t;
  //! Check if message queue is empty
  auto empty() const -> size_t;
  //! Pop message from message queue
  auto pop() -> message;
};

} // namespace x_company::geometry

#endif // INCLUDE_RECEIVER_HPP_
