#include <boost/log/trivial.hpp>
#include <boost/format.hpp>
#include <boost/version.hpp>

#define BOOST_TRACE BOOST_LOG_TRIVIAL(trace)
#define LOG(x) BOOST_TRACE << boost::format(x)

#define BOOST_VER_MAJOR (BOOST_VERSION / 100000)
#define BOOST_VER_MINOR (BOOST_VERSION / 100 % 1000)
#define BOOST_VER_PATCH (BOOST_VERSION % 100)