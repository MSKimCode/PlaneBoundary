#include <fstream>
#include <iostream>
#include <istream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace detect {

class image {
public:
  using index_t = long long int;

  bool operator()(index_t x, index_t y) const {
    return pixels.find(std::make_pair(x, y))->second;
  }

  auto size() const -> std::pair<index_t, index_t> {
    return std::make_pair(x_dim, y_dim);
  }

  image(index_t x_dim, index_t y_dim) : x_dim{x_dim}, y_dim{y_dim} {}

  void open(std::string filename) {
    std::ifstream f(filename);

    if (f.fail()) {
      std::cerr << "Cannot open file!" << std::endl;
      return;
    }

    long long int x, y;
    int p;

    while (f >> x >> y >> p) {
      pixels.insert(std::make_pair(std::make_pair(x, y), p));
    }
  }

private:
  index_t x_dim, y_dim;
  std::map<std::pair<index_t, index_t>, bool> pixels;
};

} // namespace detect
