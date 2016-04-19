// -*- C++ -*-
#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H

// c junk
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cstdio>

// c++ junk
#include <array>
#include <vector>
#include <list>
#include <string>
#include <algorithm>
#include <chrono>
#include <random>
#include <fstream>

// stlib includes
#include <stlib/geom/orq/Octree.h>

typedef std::array<double, 3> Point;
typedef stlib::geom::BBox<double, 3> BoundingBox;

// This is a little utility function that can be used to suppress any
//  compiler warnings about unused variables.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
template <class T>
void ignoreUnusedVariable(T & t) {
}
#pragma GCC diagnostic pop

/*// This is a little utility macro that can be useful for debugging.
#define debug(s, ...)                                           \
  do {                                                          \
    fprintf (stderr, "(%-20s:%40s:%4d) -- " s "\n",             \
             __FILE__, __func__, __LINE__, ##__VA_ARGS__);      \
    fflush (stderr);                                            \
  } while (0)*/

namespace Utilities {

void
verifyThatDirectoryExists(const std::string & path) {
  std::ifstream test(path);
  if ((bool)test == false) {
    fprintf(stderr, "Error, cannot find directory at %s.  "
            "Please make it yourself (\"mkdir %s\")\n",
            path.c_str(), path.c_str());
    exit(1);
  }
}

size_t
interpolateNumberLinearlyOnLogScale(const size_t lower,
                                    const size_t upper,
                                    const unsigned int numberOfPoints,
                                    const unsigned int pointIndex) {
  const double percent =
    pointIndex / double(numberOfPoints - 1);
  const double power = std::log10(lower) +
    percent * (std::log10(upper) - std::log10(lower));
  return std::pow(10., power);
}

}

// some ascii colors
#define RESET               "\033[0m"

#define BOLD_ON             "\033[1m"
#define I_ON                "\033[3m"
#define U_ON                "\033[4m"
#define INVERSE_ON          "\033[7m"
#define STRIKETHROUGH_ON    "\033[9m"
#define BOLD_OFF            "\033[21m"
#define I_OFF               "\033[23m"
#define U_OFF               "\033[24m"
#define INVERSE_OFF         "\033[27m"
#define STRIKETHROUGH_OFF   "\033[29m"

#define FG_BLACK            "\033[30m"
#define FG_RED              "\033[31m"
#define FG_GREEN            "\033[32m"
#define FG_YELLOW           "\033[33m"
#define FG_BLUE             "\033[34m"
#define FG_MAGENTA          "\033[35m"
#define FG_CYAN             "\033[36m"
#define FG_WHITE            "\033[37m"
#define FG_DEFAULT          "\033[39m"

#define BG_BLACK            "\033[40m"
#define BG_RED              "\033[41m"
#define BG_GREEN            "\033[42m"
#define BG_YELLOW           "\033[43m"
#define BG_BLUE             "\033[44m"
#define BG_MAGENTA          "\033[45m"
#define BG_CYAN             "\033[46m"
#define BG_WHITE            "\033[47m"
#define BG_DEFAULT          "\033[49m"

#endif // COMMON_DEFINITIONS_H
