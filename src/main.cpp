#include "header.hpp"
#include <iomanip>
#define DEBUG
#include <worm/worm.hpp>

int main(int argc, char **argv) {
  detect::image img(328, 270);
  img.open("blobs.dat");
  std::cout << "Loaded!" << std::endl;

  auto pp = std::make_pair(30, 200);
  // auto pp = std::make_pair(108, 254);

  std::cout << '(' << pp.first << ", " << pp.second << ')' << std::endl;
  worm::TraceBoundary<detect::image, worm::Clockwise> trace(img, pp, 1000);
  auto bd = trace.run();

  std::ofstream f("bd.out", std::ios::trunc);

  for (auto [x, y] : bd) {
    f << x << ' ' << y << std::endl;
  }
}
