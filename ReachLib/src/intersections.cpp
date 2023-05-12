/*
This file is part of Reach-RI.

Reach-RI is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
TUM, either version 3 of the License, or
(at your option) any later version.

Reach-RI is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details: https://www.gnu.org/licenses/.
*/

#include "intersections.hpp"

namespace occupancy_containers {
namespace intersections {
double segmentsDistance(const Capsule& c1, const Capsule& c2) {
  Point d1 = c1.p2_ - c1.p1_;
  Point d2 = c2.p2_ - c2.p1_;
  Point r = c2.p1_ - c1.p1_;
  double a = Point::inner_dot(d1, d1);
  double e = Point::inner_dot(d2, d2);
  double f = Point::inner_dot(d2, r);
  double epsilon = 1e-6;

  double t = 0.0;
  double s = 0.0;
  // [p1,q1] is actually a point && [p2,q2] is actually a point
  if (a <= epsilon && e <= epsilon) {
    // distance = norm(p2-p1)
    return Point::inner_dot(r, r);
  } else {
    double c = Point::inner_dot(d1, r);
    if (a <= epsilon) {
      s = 0;
      t = clamp(f/e, 0.0, 1.0);
    } else if (e < epsilon) {
      t = 0;
      s = clamp(-c/a, 0.0, 1.0);
    } else {
      double b = Point::inner_dot(d1, d2);
      double denom = a*e - b*b;
      if (denom != 0) {
        s = clamp((b*f - c*e)/denom, 0.0, 1.0);
      } else {
        s = 0;
      }
      t = (b*s + f)/e;
      if (t < 0) {
        t = 0;
        s = clamp(-c/a, 0.0, 1.0);
      } else if (t > 1) {
        t = 1;
        s = clamp((b - c)/a, 0.0, 1.0);
      }
    }
    Point closest_point_1 = get_point_from_line_segment(c1, static_cast<double>(s));
    Point closest_point_2 = get_point_from_line_segment(c2, static_cast<double>(t));
    return Point::norm(closest_point_1, closest_point_2);
  }
}
}  //  namespace intersections
}  //  namespace occupancy_containers