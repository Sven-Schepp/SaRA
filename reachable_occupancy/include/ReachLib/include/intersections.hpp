/*
This file is part of Reach-RI.

Reach-RI is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
TUM, either version 3 of the License, or
(at your option) any later version.

Reach-RI is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see https://www.gnu.org/licenses/.
*/

#include <string>
#include <vector>

#include "capsule.hpp"
#include "cylinder.hpp"
#include "occupancy.hpp"
#include "occupancy_container.hpp"
#include "point.hpp"
#include "sphere.hpp"

#ifndef REACH_LIB_INCLUDE_INTERSECTIONS_HPP_
#define REACH_LIB_INCLUDE_INTERSECTIONS_HPP_

namespace occupancy_containers {
//! \namespace Defines intersection functions among all occupancy_containers.
//! If a new occupancy_container type is defined, intersection functions with
//! among its own type, Point-type, and any other occupancy_container type must be defined.
namespace intersections {

//! \typedef Defines a shortcut to the Cylinder class
typedef cylinder::Cylinder Cylinder;

//! \typedef Defines a shortcut to the Capsule class
typedef capsule::Capsule Capsule;

//! \typedef Defines a shortcut to the Sphere class
typedef sphere::Sphere Sphere;

//! \brief Limits the value of a floating point variable by applying
//! upper and lower bounds.
//! \param[in] value Unclamped value.
//! \param[in] lower Lower value bound.
//! \param[in] upper Upper value bound.
double clamp(double value, double lower = 0.0, double upper = 1.0) {
  if (value > upper) {
    return upper;
  } else if (value < lower) {
    return lower;
  } else {
    return value;
  }
}

//! \brief Calculates the shortest distance between a Cartesian point (x, y, z)
//! and a finite line segents.
//! \param[in] c1 A Capsule object defining a line segment
//! \param[in] c2 A Capsule object defining a line segment
double point_line_segment_dist(const Point& p, const Capsule& c) {
  // https://de.mathworks.com/matlabcentral/answers/260593-distance-between-points-and-a-line-segment-in-3d
  // Vector from start to end of segment
  Point se = c.p2_ - c.p1_;
  // Length of segment
  double dse = Point::norm(se);
  // Distance from start point to point
  Point sp = p - c.p1_;
  double dsp = Point::norm(sp);
  // Distance from end point to point
  Point ep = p - c.p2_;
  double dep = Point::norm(ep);
  // Type 1: Point closest to end point
  if (sqrt(pow(dse, 2) + pow(dep, 2)) <= dsp) {
    return dep;
  }
  // Type 2: Point closest to start point
  if (sqrt(pow(dse, 2) + pow(dsp, 2)) <= dep) {
    return dsp;
  }
  // Type 3: Point in between start and end point
  return (Point::norm(Point::cross(se, ep))/dse);
}

Point get_point_from_line_segment(const Capsule& c, double t) {
  return Point(c.p1_.x + (c.p2_.x - c.p1_.x)*t,
               c.p1_.y + (c.p2_.y - c.p1_.y)*t,
               c.p1_.z + (c.p2_.z - c.p1_.z)*t);
}

//! \brief Calculates the shortest distance between two finite line segents.
//! \param[in] c1 A Capsule object defining a line segment
//! \param[in] c2 A Capsule object defining a line segment
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

double min_segment_distance(const Capsule& c1, const Capsule& c2) {
  // Calculate denominator
  Point A = c1.p2_ - c1.p1_;
  Point B = c2.p2_ - c2.p1_;
  double magA = Point::norm(A);
  double magB = Point::norm(B);

  Point _A(A.x/magA, A.y/magA, A.z/magA);
  Point _B(B.x/magB, B.y/magB, B.z/magB);

  Point cross = Point::cross(_A, _B);
  double denom = std::pow(Point::norm(cross), 2);

  // If Capsules are parrallel denom = 0
  if (denom == 0) {
    double d0 = Point::inner_dot(_A, c2.p1_ - c1.p1_);
    double d1 = Point::inner_dot(_A, c2.p2_ - c1.p1_);

    if (d0 <= 0 >= d1) {
      if (std::abs(d0) < std::abs(d1)) {
        return Point::norm(c1.p1_ - c2.p1_);
      } else {
        return Point::norm(c1.p1_ - c2.p2_);
      }
    } else if (d0 >= magA <= d1) {
      if (std::abs(d0) < std::abs(d1)) {
        return Point::norm(c1.p2_ - c2.p1_);
      } else {
        return Point::norm(c1.p2_ - c2.p2_);
      }
    } else {
      return Point::norm(Point(_A.x*d0, _A.y*d0, _A.z*d0) + c1.p1_ - c2.p1_);
    }
  } else {
    Point t = c2.p1_ - c1.p1_;
    double detA = Point::determinant3x3(t, _B, cross); // Maybe switch A, B
    double detB = Point::determinant3x3(t, _A, cross);

    double t0 = detA/denom;
    double t1 = detB/denom;

    Point pA = c1.p1_ + Point(_A.x*t0, _A.y*t0, _A.z*t0);
    Point pB = c2.p1_ + Point(_B.x*t1, _B.y*t1, _B.z*t1);

    if (t0 < 0) {
      pA = c1.p1_;
    } else if (t0 > magA) {
      pA = c1.p2_;
    } else if (t1 < 0) {
      pB = c2.p1_;
    } else if (t1 > magB) {
      pB = c2.p2_;
    }

    if (t0 < 0 || t0 > magA) {
      double dot = Point::inner_dot(_B, pA - c2.p1_);
      if (dot < 0) {
        dot = 0;
      } else if (dot > magB) {
        dot = magB;
      }
      pB = c2.p1_ + Point(_B.x*dot, _B.y*dot, _B.z*dot);

    }
    if (t1 < 0 || t1 > magB) {
      double dot = Point::inner_dot(_A, pB - c1.p1_);
      if (dot < 0) {
        dot = 0;
      } else if (dot > magA) {
        dot = magA;
      }
      pA = c1.p1_ + Point(_A.x*dot, _A.y*dot, _A.z*dot);
    }
    return Point::norm(pA - pB);
  }
}

//! \brief Calculates whether the shortest distance between two capsules
//! \param[in] c1 An object of type Capsule
//! \param[in] c2 An object of type Capsule
double capsule_capsule_dist(const Capsule& c1, const Capsule& c2) {
  // First check if any capsule is a sphere
  bool is_sphere1 = c1.p1_ == c1.p2_;
  bool is_sphere2 = c2.p1_ == c2.p2_;
  // Sphere collision check
  if (is_sphere1 && is_sphere2) {
    return Point::norm(c1.p1_, c2.p1_) - c1.r_ - c2.r_;
  // Sphere and capsule collision check
  } else if (is_sphere1 && !is_sphere2) {
    return point_line_segment_dist(c1.p1_, c2) - c1.r_ - c2.r_;
  // Sphere and capsule collision check
  } else if (!is_sphere1 && is_sphere2) {
    return point_line_segment_dist(c2.p1_, c1) - c1.r_ - c2.r_;
  // Capsule and capsule collision
  } else {
    // return segmentsDistance(c1, c2) - c1.r_ - c2.r_;
    // return c_c_distance(c1, c2);
    return min_segment_distance(c1, c2) - c1.r_ - c2.r_;
  }
}

//! \brief Determines whether there exists an intersection between two capsules
//! \param[in] c1 An object of type Capsule
//! \param[in] c2 An object of type Capsule
bool capsule_capsule_intersection(const Capsule& c1, const Capsule& c2) {
  return capsule_capsule_dist(c1, c2) < 0.0;
}

//! \brief Calculates the closest distance between two VERTICALLY extruded cylinders.
//! \param[in] c1 An object of type Cylinder
//! \param[in] c2 An object of type Cylinder
double cylinder_cylinder_dist(const Cylinder& c1, const Cylinder& c2) {
  return c1.r_ + c2.r_ - Point::norm(c1.p1_ - c2.p1_);
}

//! \brief Determines whether there exists an intersection
//! between two VERTICALLY extruded cylinders.
//! \param[in] c1 An object of type Cylinder
//! \param[in] c2 An object of type Cylinder
bool cylinder_cylinder_intersection(const Cylinder& c1, const Cylinder& c2) {
  return cylinder_cylinder_dist(c1, c2) < 0.0;
}

//! \brief Calculates the closest distance between an object of type cylinder
//! and an object of type capsule where cylinders are approximated as capsules
//! \param[in] cy An object of type Cylinder
//! \param[in] ca An object of type Capsule
double cylinder_capsule_dist(const Cylinder& cy, const Capsule& ca) {
  Capsule cyc = Capsule(cy.p1_, cy.p2_, cy.r_);
  return capsule_capsule_dist(cyc, ca);
}

//! \brief Determines whether an object of type Cylinder
//! and an object of type Capsule intersect.
//! \param[in] cy An object of type Cylinder
//! \param[in] ca An object of type Capsule
bool cylinder_capsule_intersection(const Cylinder& cy, const Capsule& ca) {
  return cylinder_capsule_dist(cy, ca) < 0.0;
}

//! \brief Calculates the closest distance between an object of type Sphere
//! and an object of type Capsule.
//! \param[in] cy An object of type Cylinder
//! \param[in] ca An object of type Capsule
double sphere_capsule_dist(const Sphere& s, const Capsule& c) {
  return capsule_capsule_dist(static_cast<Capsule>(s), c);
}

//! \brief Determines whether an object of type Sphere
//! and an object of type Capsule intersect.
//! \param[in] cy An object of type Cylinder.
//! \param[in] ca An object of type Capsule.
double sphere_capsule_intersection(const Sphere& s, const Capsule& c) {
  return capsule_capsule_intersection(static_cast<Capsule>(s), c);
}

//! \brief Calculates the closest distance between two objects of type Sphere.
//! \param[in] s1 An object of type Sphere.
//! \param[in] s2 An object of type Sphere.
double sphere_sphere_dist(const Sphere& s1, const Sphere& s2) {
  return s1.r_ + s2.r_ - Point::norm(s1.p_, s2.p_);
}

//! \brief Determines whether two objects of type Sphere intersect.
//! \param[in] s1 An object of type Sphere.
//! \param[in] s2 An object of type Sphere.
double sphere_sphere_intersection(const Sphere& s1, const Sphere& s2) {
  return sphere_sphere_dist(s1, s2) < 0.0;
}





/*


//! \brief Determines whether any 3D point in a list lies within a capsule.
//! \param[in] c Capsule object checked for intersection
//! \param[in] points A list of points in 3D Cartesian coordinates (x, y, z)
//! \param[in] r Minmum safety distance between the point and the capsule.
//!            Used to calculate ball capsule intersection.
bool point_capsule_intersection(const Capsule& c, const std::vector<Point>& points, double r = 0.0) {
  // Check whether the capsule is a ball
  if (c.p1_ == c.p2_) {
    for (const auto& p : points) {
      return 0 <= c.r_ + r - Point::norm(c.p1_, p);
    }
  }
  for (const auto& p : points) {
    // The capsules cylinder is defined by a line segment from
    // the center of the top circle to the center of the bottom circle.
    Point d = c.p2_ - c.p1_;
    Point p1d = p - c.p1_;
    Point p2d = p - c.p2_;
    double h = Point::norm(d);
    double hsq = pow(h, 2);
    double dotprod = Point::inner_dot(p1d, d);

    // Check whether the point lies higher or lower than the line segment
    // in the cylinders coordinates
    // If the point lies outside the cylinders domain given an infinite radius
    // (point lies higher or lower than the cylinders top/bottom):
    // Check for intersections with the capsules halfspheres (the same as a sphere radius check)
    if ((dotprod < 0.0) || (dotprod > hsq)) {
      if (Point::norm(p1d) > c.r_ + r && Point::norm(p2d) > c.r_ + r) {
          return false;
      } else {
          return true;
      }
    // If the point lies next to the line segment in the cylinders coordinates:
    // check if the shortes point to line distance exceeds the cylinders radius
    // if yes: no intersection
    } else {
      double rsq = pow(c.r_ + r, 2);
      double dsq = Point::inner_dot(p1d, p1d) - ((dotprod*dotprod)/hsq);
      if (dsq > rsq) {
          return false;
      } else {
          return true;
      }
    }
  }
}

//! \brief Determines if the end-caps and the cylinder of Capsule c2 intersects with
//!        any part of capsule c1
//! \param[in] c1 Capsule object one in intersection check
//! \param[in] c2 Capsule object two in intersection check
bool one_sided_capsule_intersection(const Capsule& c1, const Capsule& c2) {
  // Check whether there is an intersection between any of the half spheres
  if (point_capsule_intersection(c1, std::vector<Point> {c2.p1_}, c2.r_) == true ||
      point_capsule_intersection(c1, std::vector<Point> {c2.p2_}, c2.r_) == true) {
    return true;
  } else {
    // Check whether there exists an intersection between the cylinder of c2
    // and the capsule c1 by calclating the shortest distance
    // between two line segments
    Point e1 = c1.p2_ - c1.p1_;
    Point e2 = c2.p2_ - c2.p1_;
    // Normal vector of the plane given by the line segments
    Point n = Point::cross(e1, e2);

    Point diff;
    // Which points are higher in z direction?
    // Determines the sign of the distance vector (should always be positive).
    if (c1.p1_.z <= c2.p1_.z) {
        diff = c1.p1_ - c2.p1_;
    } else {
        diff = c2.p1_ - c1.p1_;
    }
    // Project the difference vector onto the normal vector
    // and determine whether the projetced vector exceeds the sum of radii
    // -> If the distance between two lines is greater than the sum of both capsule radii:
    // -> no collision.
    double d = Point::inner_dot(n, diff) / Point::norm(n);
    if (std::abs(d) <= c1.r_ + c2.r_) {
        return true;
    } else {
        return false;
    }
  }
}

//! \brief Determines whether capsule c1 and capsule c2 intersect in any point.
//! \param[in] c1 Capsule object checked for intersection
//! \param[in] c2 Capsule object checked for intesection
bool d_capsule_capsule_intersection(Capsule c1, Capsule c2) {
  bool is_ball1 = c1.p1_ == c1.p2_;
  bool is_ball2 = c2.p1_ == c2.p2_;

  if (!is_ball1 && !is_ball2) {
    return one_sided_capsule_intersection(c1, c2);
  } else {
    return point_capsule_intersection(c1, std::vector<Point> {c2.p1_, c2.p2_}, c2.r_);
  }
}

*/
/*

//! \brief Determines if the end-caps and the cylinder of Capsule c2 intersects with
//!        any part of capsule c1
//! \param[in] c1 Capsule object one in intersection check
//! \param[in] c2 Capsule object two in intersection check
bool one_sided_capsule_intersection(const Capsule& c1, const Capsule& c2) {
  // Check whether there is an intersection between any of the half spheres
  if (point_capsule_intersection(c1, std::vector<Point> {c2.p1_}, c2.r_) == true ||
      point_capsule_intersection(c1, std::vector<Point> {c2.p2_}, c2.r_) == true) {
    return true;

  } else {
    // Check whether there exists an intersection between the cylinder of c2
    // and the capsule c1 by calclating the shortest distance
    // between two line segments
    Point e1 = c1.p2_ - c1.p1_;
    Point e2 = c2.p2_ - c2.p1_;
    // Normal vector of the plane given by the line segments
    Point n = Point::cross(e1, e2);

    Point diff;
    // Which points are higher in z direction?
    // Determines the sign of the distance vector (should always be positive).
    if (c1.p1_.z <= c2.p1_.z) {
        diff = c1.p1_ - c2.p1_;
    } else {
        diff = c2.p1_ - c1.p1_;
    }
    // Project the difference vector onto the normal vector
    // and determine whether the projetced vector exceeds the sum of radii
    // -> If the distance between two lines is greater than the sum of both capsule radii:
    // -> no collision.
    double d = Point::inner_dot(n, diff) / Point::norm(n);
    if (std::abs(d) <= c1.r_ + c2.r_) {
        return true;
    } else {
        return false;
    }
  }
}

//! \brief Determines whether capsule c1 and capsule c2 intersect in any point.
//! \param[in] c1 Capsule object checked for intersection
//! \param[in] c2 Capsule object checked for intesection
bool capsule_capsule_intersection(Capsule c1, Capsule c2) {
  bool is_ball1 = c1.p1_ == c1.p2_;
  bool is_ball2 = c2.p1_ == c2.p2_;

  if (!is_ball1 && !is_ball2) {
    return one_sided_capsule_intersection(c1, c2) &&
           one_sided_capsule_intersection(c2, c1);
  } else {
    return point_capsule_intersection(c1, std::vector<Point> {c2.p1_, c2.p2_}) &&
           point_capsule_intersection(c2, std::vector<Point> {c1.p1_, c1.p2_});
  }
}

//! \brief Determines whether an object of type capsule and an object of type cylinder intersect.
//! \param[in] ca An object of type Capsule
//! \param[in] cy An object of type Cylinder
bool cylinder_capsule_intersection(const Capsule& ca, const Cylinder& cy) {
    // Calculate the shortest distance between two line segments
    // (of the cylinder and the capsule)
    Point e1 = ca.p2_ - ca.p1_;
    Point e2 = cy.p2_ - cy.p1_;
    // Normal vector of the plane given by the line segments
    Point n = Point::cross(e1, e2);

    Point diff;
    // Which points are higher in z direction?
    // Determines the sign of the distance vector (should always be positive).
    if (ca.p1_.z <= cy.p1_.z) {
        diff = ca.p1_ - cy.p1_;
    } else {
        diff = ca.p1_ - cy.p1_;
    }
    // Project the difference vector onto the normal vector
    // and determine whether the projetced vector exceeds the sum of radii
    // -> If the distance between two lines is greater than the sum of both radii:
    // -> no collision.
    double d = Point::inner_dot(n, diff) / Point::norm(n);
    if (std::abs(d) <= ca.r_ + cy.r_) {
        return true;
    } else {
        return false;
    }
}

//! \brief Determines whether any 3D point in a list lies within a cylinder.
//! \param[in] c Cylinder object checked for intersection
//! \param[in] points A list of points in 3D Cartesian coordinates (x, y, z)
bool point_cylinder_intersection(const Cylinder& c, std::vector<Point> points) {
  for (const auto& p : points) {
    // The cylinder is defined by a line segment from
    // the center of the top circle to the center of the bottom circle.
    Point d = c.p2_ - c.p1_;
    Point p1d = p - c.p1_;
    Point p2d = p - c.p2_;
    double h = Point::norm(d);
    double hsq = pow(h, 2);
    double dotprod = Point::inner_dot(p1d, d);

    // Check whether the point lies higher or lower than the line segment
    // in the cylinders coordinates
    // If the point lies outside the cylinders domain given an infinite radius
    // (point lies higher or lower than the cylinders top/bottom):
    // -> No intersection.
    if ((dotprod < 0.0) || (dotprod > hsq)) {
        return false;
    // If the point lies next to the line segment in the cylinders coordinates:
    // check if the shortes point to line distance exceeds the cylinders radius
    // if yes: no intersection
    } else {
      double rsq = pow(c.r_, 2);
      double dsq = Point::inner_dot(p1d, p1d) - ((dotprod*dotprod)/hsq);
      if (dsq > rsq) {
          return false;
      } else {
          return true;
      }
    }
  }
}

//! \brief Returns true if there is an intersection between two VERTICALLY extruded cylinders.
//! \param[in] c1 An object of type Cylinder
//! \param[in] c2 An object of type Cylinder
bool cylinder_cylinder_intersection(const Cylinder& c1, const Cylinder& c2) {
  return 0 <= c1.r_ + c2.r_ - Point::norm(c1.p1_ - c2.p1_);
}

double cylinder_capsule_dist(const Cylinder& cy, const Capsule& ca) {
  Capsule cyl_cap = static_cast<Capsule> (cy);
  double top = 0.0;
  double bottom = 0.0;
  // Which cylinder point has the greater and lesser z value
  if (cyl_cap.p1_.z > cyl_cap.p2_.z) {
    top = cyl_cap.p1_.z;
    bottom = cyl_cap.p2_.z;
  } else {
    top = cyl_cap.p2_.z;
    bottom = cyl_cap.p1_.z;
  }
  // Find the closest point from capsule to cylinder
  double d1 = point_line_segment_dist(ca.p1_, cyl_cap);
  double d2 = point_line_segment_dist(ca.p2_, cyl_cap);
  // If there is no intersection between a capsulized cylinder and ca
  double dist = capsule_capsule_dist(cyl_cap, ca);
  if (!(dist < cyl_cap.r_ + ca.r_)) {
    // Check the min distance using the closest point
    if (d1 < d2) {
      // Check whether the point lies higher, lower, or between the cylinder plates
      if (ca.p1_.z > top || ca.p1_.z < bottom) {
        return dist - ca.r_;
      } else {
        return dist - cyl_cap.r_ - ca.r_;
      }
    } else {
      // Check whether the point lies higher, lower, or between the cylinder plates
      if (ca.p2_.z > top || ca.p2_.z < bottom) {
        return dist - ca.r_;
      } else {
        return dist - cyl_cap.r_ - ca.r_;
      }
    }
  } else {
    // Check the min distance using the closest point
    if (d1 < d2) {
      // Check whether the point lies higher, lower, or between the cylinder plates
      if (ca.p1_.z > top || ca.p1_.z < bottom) {
        return dist - ca.r_;
      } else {
        return dist - cyl_cap.r_ - ca.r_;
      }
    } else {
      // Check whether the point lies higher, lower, or between the cylinder plates
      if (ca.p2_.z > top || ca.p2_.z < bottom) {
        return dist - ca.r_;
      } else {
        return dist - cyl_cap.r_ - ca.r_;
      }
    }
  }

  // Calculate the shortest distance between two line segments
  // (of the cylinder and the capsule)
  Point e1 = ca.p2_ - ca.p1_;
  Point e2 = cy.p2_ - cy.p1_;
  // Normal vector of the plane given by the line segments
  Point n = Point::cross(e1, e2);

  Point diff;
  // Which points are higher in z direction?
  // Determines the sign of the distance vector (should always be positive).
  if (ca.p1_.z <= cy.p1_.z) {
      diff = ca.p1_ - cy.p1_;
  } else {
      diff = ca.p1_ - cy.p1_;
  }
  // Project the difference vector onto the normal vector
  // and determine whether the projetced vector exceeds the sum of radii
  // -> If the distance between two lines is greater than the sum of both radii:
  // -> no collision.
  double d = Point::inner_dot(n, diff) / Point::norm(n);
  return ca.r_ + cy.r_ - std::abs(d);
}

*/

}  //  namespace intersections
}  //  namespace occupancy_containers
#endif  //  REACH_LIB_INCLUDE_INTERSECTIONS_HPP_
