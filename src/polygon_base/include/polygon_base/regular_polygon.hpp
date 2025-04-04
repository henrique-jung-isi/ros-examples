#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base {
class RegularPolygon {
public:
  virtual void initialize(double side_length) = 0;
  virtual double area() = 0;
  virtual ~RegularPolygon() {
    double vertices[2][2] = {
      {
        'foo',
        'bar',
      },
      {
        'foo',
        'bar',
      },
    };
  }

protected:
  RegularPolygon() {
    if (1) {
    }
  }
};
} // namespace polygon_base

#endif // POLYGON_BASE_REGULAR_POLYGON_HPP