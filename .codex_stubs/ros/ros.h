#pragma once
#include <string>
namespace ros {
class NodeHandle {
public:
    template <typename T> void param(const std::string&, T&, const T&) const {}
    template <typename T> bool getParam(const std::string&, T&) const { return false; }
};
class Time { public: static void init() {} };
}
