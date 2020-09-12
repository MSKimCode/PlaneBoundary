#include <iomanip>
#include "header.hpp"
#include <worm/worm.hpp>

int main(int argc, char **argv) {
  detect::image img(328, 270);
  img.open("blobs.dat");

  /*
  for (int y = 0; y < 10; ++y) {
    for (int x = 0; x < 40; ++x) {
      std::cout << img(x, y);
    }
    std::cout << std::endl;
  }
  */
  worm::TraceBoundary<detect::image, worm::Clockwise> trace(img, {100, 120}, 0);
  auto bd = trace.run();

  std::ofstream f("bd.out");

  for (auto [x, y] : bd) {
    f << x << ' ' << y << std::endl;
  }
}
