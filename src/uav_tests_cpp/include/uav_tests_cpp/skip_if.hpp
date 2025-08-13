#pragma once
#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include <algorithm>

#include <uav_control_cpp/not_implemented.hpp>  // aday tipimiz

namespace testutil
{
inline bool looks_like_todo_msg(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s.find("todo") != std::string::npos
      || s.find("not implemented") != std::string::npos
      || s.find("implement") != std::string::npos;
}

/** 
 * İçine koyduğun ifade NotImplemented/“TODO” kaynaklı patlarsa -> GTEST_SKIP.
 * Diğer istisnalar gerçek FAIL olarak yukarı fırlatılır.
 */
#define SKIP_IF_NOT_IMPLEMENTED(block)                                          \
  do {                                                                          \
    try {                                                                       \
      block;                                                                    \
    } catch (const uav::NotImplemented& e) {                                    \
      GTEST_SKIP() << "Not implemented by candidate: " << e.what();             \
    } catch (const std::logic_error& e) {                                       \
      if (testutil::looks_like_todo_msg(e.what())) {                            \
        GTEST_SKIP() << "Not implemented (TODO): " << e.what();                 \
      } else {                                                                  \
        throw;                                                                  \
      }                                                                         \
    }                                                                           \
  } while (0)

} // namespace testutil
