#pragma once
#include <stdexcept>
#include <string>

namespace uav
{
/** Aday henüz yazmadıysa bunu fırlatır. Testler bunu SKIP'e çevirir. */
struct NotImplemented : public std::logic_error
{
  explicit NotImplemented(const std::string& what_arg)
    : std::logic_error(what_arg) {}
};
} // namespace uav
