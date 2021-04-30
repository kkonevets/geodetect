#include <boost/spirit/include/qi.hpp>
#include <memory>
#include <stdexcept>
#include <vector>

#include "receiver.hpp"

namespace qi = boost::spirit::qi;

namespace x_company::geometry {

void trigger_listener::operator()(std::string const &payload, int backend_pid) {
  static bool initialized = false;

  // start receiving messages only after geoindex::initialize()
  if (!initialized) {
    if (conn().backendpid() == backend_pid)
      initialized = true;
    else
      return;
  }

  // e.g. payload  is "(7634,gz_polygon,INSERT)", we will parse that
  message msg;
  auto first = payload.begin();
  qi::rule<std::string::const_iterator, std::string()> r =
      +(qi::char_ - qi::char_(",)"));
  bool success = qi::parse(first, payload.end(),
                           '(' >> qi::int_ >> ",gz_" >> r >> "," >> r >> ')',
                           msg.id, msg.shape, msg.op);

  if (!success || first != payload.end()) {
    std::cerr << "Parsing failed:" + payload << std::endl;
    throw std::logic_error("Parsing failed:" + payload);
  }
  // add message to message queue
  msgs_.push(std::move(msg));
}

auto trigger_listener::size() const -> size_t { return msgs_.size(); }

auto trigger_listener::empty() const -> size_t { return msgs_.empty(); }

auto trigger_listener::pop() -> message {
  auto msg = std::move(msgs_.front());
  msgs_.pop();
  return msg;
}

} // namespace x_company::geometry
