#ifndef BATCH_VISUALIZER_H
#define BATCH_VISUALIZER_H

#define RAY_THICKNESS 0.005
#define POINT_SIZE 0.05

#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <boost/optional.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>


/* GeometryTools //{ */

/* Ray //{ */
class Ray {
public:
  Ray();
  ~Ray();
  Ray(Eigen::Vector3d p1, Eigen::Vector3d p2);

  Eigen::Vector3d p1;
  Eigen::Vector3d p2;
  Eigen::Vector3d direction;

  static Ray twopointCast(Eigen::Vector3d pointFrom, Eigen::Vector3d pointTo) {
    return Ray(pointFrom, pointTo);
  }
  static Ray directionCast(Eigen::Vector3d origin, Eigen::Vector3d direction) {
    return Ray(origin, origin + direction);
  }
};

Ray::Ray() {
  this->p1 = Eigen::Vector3d(0.0, 0.0, 0.0);
  this->p2 = Eigen::Vector3d(0.0, 0.0, 0.0);
}

Ray::~Ray() {
}

Ray::Ray(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  this->p1        = p1;
  this->p2        = p2;
  this->direction = p2 - p1;
}
//}

/* Plane //{ */
class Plane {
public:
  Plane();
  ~Plane();
  Plane(Eigen::Vector3d point, Eigen::Vector3d normal);

  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  double          d = 0;

  boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
};

Plane::Plane() {
}

Plane::Plane(Eigen::Vector3d point, Eigen::Vector3d normal) {
  this->point  = point;
  this->normal = normal;
  this->d      = -(normal.dot(point));
}

Plane::~Plane() {
}

boost::optional<Eigen::Vector3d> Plane::intersectionRay(Ray r, double epsilon) {
  double denom = this->normal.dot(r.p2 - r.p1);
  if (abs(denom) < epsilon) {
    return boost::optional<Eigen::Vector3d>{};
  }
  double t = this->normal.dot(this->point - r.p1) / denom;
  if (t >= 0) {
    return Eigen::Vector3d(r.p1 + t * r.direction);
  } else {
    return boost::optional<Eigen::Vector3d>{};
  }
}

//}

/* Rectangle //{ */
class Rectangle {
public:
  Rectangle();
  ~Rectangle();
  Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D);

  Eigen::Vector3d normal_vector;

  Eigen::Matrix3d basis;
  Eigen::Matrix3d projector;

  double diagonal_length;

  Plane plane;

  std::vector<Eigen::Vector3d> points;

  boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
};

Rectangle::Rectangle() {
}

Rectangle::~Rectangle() {
}

Rectangle::Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D) {

  Eigen::Vector3d v1 = B - A;
  Eigen::Vector3d v2 = D - A;

  diagonal_length = (C - A).norm();

  this->points.push_back(A);
  this->points.push_back(B);
  this->points.push_back(C);
  this->points.push_back(D);

  this->normal_vector = v1.cross(v2);
  this->normal_vector.normalize();

  if (A == B || A == C || A == D || B == C || B == D || C == D) {
    return;
  }

  this->plane = Plane(A, normal_vector);

  this->basis.col(0) << B - A;
  this->basis.col(1) << D - A;
  this->basis.col(2) << normal_vector;

  this->projector = basis * basis.transpose();
}

boost::optional<Eigen::Vector3d> Rectangle::intersectionRay(Ray r, double epsilon) {
  boost::optional<Eigen::Vector3d> intersect = this->plane.intersectionRay(r, epsilon);
  if (!intersect) {
    return intersect;
  }
  Eigen::Vector3d projection = basis.inverse() * (intersect.get() - points[0]);
  if (projection[0] >= 0.0 && projection[0] <= 1.0 && projection[1] >= 0.0 && projection[1] <= 1.0) {
    return intersect;
  }
  return boost::optional<Eigen::Vector3d>{};
}
//}

/* Cuboid //{ */
class Cuboid {
public:
  Cuboid();
  ~Cuboid();
  Cuboid(Eigen::Vector3d center, Eigen::Quaterniond orientation, Eigen::Vector3d scale);
  Cuboid(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D, Eigen::Vector3d E, Eigen::Vector3d F, Eigen::Vector3d G,
         Eigen::Vector3d H);

  std::vector<Eigen::Vector3d> vertices;
  std::vector<Rectangle>       sides;
};

Cuboid::Cuboid() {
}

Cuboid::~Cuboid() {
}

Cuboid::Cuboid(Eigen::Vector3d center, Eigen::Quaterniond orientation, Eigen::Vector3d scale) {

  Eigen::Vector3d A = orientation * Eigen::Vector3d(scale[0] / 2.0, -scale[1] / 2.0, -scale[2] / 2.0) + center;
  Eigen::Vector3d B = orientation * Eigen::Vector3d(scale[0] / 2.0, scale[1] / 2.0, -scale[2] / 2.0) + center;
  Eigen::Vector3d C = orientation * Eigen::Vector3d(scale[0] / 2.0, scale[1] / 2.0, scale[2] / 2.0) + center;
  Eigen::Vector3d D = orientation * Eigen::Vector3d(scale[0] / 2.0, -scale[1] / 2.0, scale[2] / 2.0) + center;

  Eigen::Vector3d E = orientation * Eigen::Vector3d(-scale[0] / 2.0, scale[1] / 2.0, -scale[2] / 2.0) + center;
  Eigen::Vector3d F = orientation * Eigen::Vector3d(-scale[0] / 2.0, -scale[1] / 2.0, -scale[2] / 2.0) + center;
  Eigen::Vector3d G = orientation * Eigen::Vector3d(-scale[0] / 2.0, -scale[1] / 2.0, scale[2] / 2.0) + center;
  Eigen::Vector3d H = orientation * Eigen::Vector3d(-scale[0] / 2.0, scale[1] / 2.0, scale[2] / 2.0) + center;

  Rectangle front(A, B, C, D);
  Rectangle back(E, F, G, H);
  Rectangle left(B, E, H, C);
  Rectangle right(F, A, D, G);
  Rectangle bottom(F, E, B, A);
  Rectangle top(D, C, H, G);

  sides.push_back(front);
  sides.push_back(back);
  sides.push_back(left);
  sides.push_back(right);
  sides.push_back(bottom);
  sides.push_back(top);

  this->vertices.clear();
  this->vertices.push_back(A);
  this->vertices.push_back(B);
  this->vertices.push_back(C);
  this->vertices.push_back(D);
  this->vertices.push_back(E);
  this->vertices.push_back(F);
  this->vertices.push_back(G);
  this->vertices.push_back(H);
}

Cuboid::Cuboid(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D, Eigen::Vector3d E, Eigen::Vector3d F, Eigen::Vector3d G,
               Eigen::Vector3d H) {
  this->vertices.clear();
  this->vertices.push_back(A);
  this->vertices.push_back(B);
  this->vertices.push_back(C);
  this->vertices.push_back(D);
  this->vertices.push_back(E);
  this->vertices.push_back(F);
  this->vertices.push_back(G);
  this->vertices.push_back(H);

  Rectangle front(A, B, C, D);
  Rectangle back(E, F, G, H);
  Rectangle left(B, E, H, C);
  Rectangle right(F, A, D, G);
  Rectangle bottom(F, E, B, A);
  Rectangle top(D, C, H, G);

  sides.push_back(front);
  sides.push_back(back);
  sides.push_back(left);
  sides.push_back(right);
  sides.push_back(bottom);
  sides.push_back(top);
}
//}

//}

/* VisualTools //{ */
class VisualTools {
public:
  static void visualizeRay(ros::Publisher pub, Ray ray, std::string frame);
  static void visualizePoint(ros::Publisher pub, Eigen::Vector3d p, std::string frame, double size, double r = 0.0, double g = 1.0, double b = 0.0);
  static void visualizeRect(ros::Publisher pub, Rectangle rect, std::string frame, double r = 0.0, double g = 0.7, double b = 1.0);
  static void visualizeCuboid(ros::Publisher pub, Cuboid cuboid, std::string frame, double r = 0.0, double g = 0.7, double b = 1.0);
};

/* visualizeRay */  //{
void VisualTools::visualizeRay(ros::Publisher pub, Ray ray, std::string frame) {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id    = frame;
  line_strip.header.stamp       = ros::Time::now();
  line_strip.ns                 = "ray";
  line_strip.action             = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id                 = 1;
  line_strip.type               = visualization_msgs::Marker::LINE_STRIP;

  line_strip.scale.x = 0.002;

  line_strip.color.r = 1.0;
  line_strip.color.a = 1.0;

  geometry_msgs::Point p1, p2;

  p1.x = ray.p1[0];
  p1.y = ray.p1[1];
  p1.z = ray.p1[2];

  p2.x = ray.p2[0];
  p2.y = ray.p2[1];
  p2.z = ray.p2[2];

  line_strip.points.push_back(p1);
  line_strip.points.push_back(p2);
  pub.publish(line_strip);
}
//}

/* visualizePoint */  //{
void VisualTools::visualizePoint(ros::Publisher pub, Eigen::Vector3d p, std::string frame, double size, double r, double g, double b) {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "point";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::POINTS;

  marker.scale.x = size;
  marker.scale.y = size;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  geometry_msgs::Point gp;
  gp.x = p[0];
  gp.y = p[1];
  gp.z = p[2];

  marker.points.push_back(gp);

  pub.publish(marker);
}
//}

/* visualizeRect */  //{
void VisualTools::visualizeRect(ros::Publisher pub, Rectangle rect, std::string frame, double r, double g, double b) {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id    = frame;
  line_strip.header.stamp       = ros::Time::now();
  line_strip.ns                 = "rect";
  line_strip.action             = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id                 = 1;
  line_strip.type               = visualization_msgs::Marker::LINE_STRIP;

  line_strip.scale.x = 0.002;

  line_strip.color.r = r;
  line_strip.color.g = g;
  line_strip.color.b = b;
  line_strip.color.a = 1.0;

  geometry_msgs::Point pa, pb, pc, pd;

  pa.x = rect.points[0][0];
  pa.y = rect.points[0][1];
  pa.z = rect.points[0][2];

  pb.x = rect.points[1][0];
  pb.y = rect.points[1][1];
  pb.z = rect.points[1][2];

  pc.x = rect.points[2][0];
  pc.y = rect.points[2][1];
  pc.z = rect.points[2][2];

  pd.x = rect.points[3][0];
  pd.y = rect.points[3][1];
  pd.z = rect.points[3][2];

  line_strip.points.push_back(pa);
  line_strip.points.push_back(pb);
  line_strip.points.push_back(pc);
  line_strip.points.push_back(pd);
  line_strip.points.push_back(pa);

  pub.publish(line_strip);
  ros::spinOnce();
}
//}

//}

/* BatchVisualizer //{ */

/* declaration //{ */
class BatchVisualizer {
public:
  BatchVisualizer();
  ~BatchVisualizer();
  BatchVisualizer(ros::NodeHandle rosNode, std::string frame);

  void addRay(Ray ray, Eigen::Vector4d color);
  void addPoint(Eigen::Vector3d p, Eigen::Vector4d color);
  void addRect(Rectangle rect, Eigen::Vector4d color);
  void addCuboid(Eigen::Vector3d pos, Eigen::Quaterniond ori, Eigen::Vector3d sca, Eigen::Vector4d color);

  void clear();
  void publish();

private:
  std::string                       frame;
  std::vector<geometry_msgs::Point> vertices;
  std::vector<std_msgs::ColorRGBA>  vertex_colors;
  ros::Publisher                    visual_pub;

  int triangle_count = 0;
};
//}

/* constructors */  //{
BatchVisualizer::BatchVisualizer() {
}

BatchVisualizer::~BatchVisualizer() {
}

BatchVisualizer::BatchVisualizer(ros::NodeHandle rosNode, std::string frame) {
  this->frame      = frame;
  this->visual_pub = rosNode.advertise<visualization_msgs::Marker>("/radiation/visualizer", 1);
}
//}

/* addRay */  //{
void BatchVisualizer::addRay(Ray ray, Eigen::Vector4d color) {

  std_msgs::ColorRGBA vertex_color;
  vertex_color.r = color[0];
  vertex_color.g = color[1];
  vertex_color.b = color[2];
  vertex_color.a = color[3];

  geometry_msgs::Point p1, p2, p3, p4;

  p1.x = ray.p1[0] + RAY_THICKNESS / 2;
  p1.y = ray.p1[1] + RAY_THICKNESS / 2;
  p1.z = ray.p1[2];

  p2.x = ray.p1[0] - RAY_THICKNESS / 2;
  p2.y = ray.p1[1] - RAY_THICKNESS / 2;
  p2.z = ray.p1[2];

  p3.x = ray.p2[0] + RAY_THICKNESS / 2;
  p3.y = ray.p2[1] + RAY_THICKNESS / 2;
  p3.z = ray.p2[2];

  p4.x = ray.p2[0] - RAY_THICKNESS / 2;
  p4.y = ray.p2[1] - RAY_THICKNESS / 2;
  p4.z = ray.p2[2];

  vertices.push_back(p1);
  vertices.push_back(p2);
  vertices.push_back(p3);
  vertices.push_back(p4);
  vertices.push_back(p3);
  vertices.push_back(p1);

  triangle_count += 2;

  for (int i = 0; i < 6; i++) {
    vertex_colors.push_back(vertex_color);
  }
}
//}

/* addPoint */  //{
void BatchVisualizer::addPoint(Eigen::Vector3d p, Eigen::Vector4d color) {

  std_msgs::ColorRGBA vertex_color;
  vertex_color.r = color[0];
  vertex_color.g = color[1];
  vertex_color.b = color[2];
  vertex_color.a = color[3];

  geometry_msgs::Point p1, p2, p3, p4;
  p1.x = p[0] + POINT_SIZE / 2;
  p1.y = p[1] + POINT_SIZE / 2;
  p1.z = p[2];

  p2.x = p[0] - POINT_SIZE / 2;
  p2.y = p[1] - POINT_SIZE / 2;
  p2.z = p[2];

  p3.x = p[0] + POINT_SIZE / 2;
  p3.y = p[1] - POINT_SIZE / 2;
  p3.z = p[2];

  p4.x = p[0] - POINT_SIZE / 2;
  p4.y = p[1] + POINT_SIZE / 2;
  p4.z = p[2];


  vertices.push_back(p1);
  vertices.push_back(p3);
  vertices.push_back(p2);
  vertices.push_back(p2);
  vertices.push_back(p1);
  vertices.push_back(p4);

  triangle_count += 2;

  for (int i = 0; i < 6; i++) {
    vertex_colors.push_back(vertex_color);
  }
}
//}

/* addRect */  //{
void BatchVisualizer::addRect(Rectangle rect, Eigen::Vector4d color) {

  std_msgs::ColorRGBA vertex_color;
  vertex_color.r = color[0];
  vertex_color.g = color[1];
  vertex_color.b = color[2];
  vertex_color.a = color[3];

  geometry_msgs::Point pa, pb, pc, pd;

  pa.x = rect.points[0][0];
  pa.y = rect.points[0][1];
  pa.z = rect.points[0][2];

  pb.x = rect.points[1][0];
  pb.y = rect.points[1][1];
  pb.z = rect.points[1][2];

  pc.x = rect.points[2][0];
  pc.y = rect.points[2][1];
  pc.z = rect.points[2][2];

  pd.x = rect.points[3][0];
  pd.y = rect.points[3][1];
  pd.z = rect.points[3][2];

  vertices.push_back(pa);
  vertices.push_back(pb);
  vertices.push_back(pc);

  vertices.push_back(pd);
  vertices.push_back(pa);
  vertices.push_back(pc);

  triangle_count += 2;

  for (int i = 0; i < 6; i++) {
    vertex_colors.push_back(vertex_color);
  }
}
//}

/* addCuboid */  //{
void BatchVisualizer::addCuboid(Eigen::Vector3d pos, Eigen::Quaterniond ori, Eigen::Vector3d sca, Eigen::Vector4d color) {

  std_msgs::ColorRGBA vertex_color;
  vertex_color.r = color[0];
  vertex_color.g = color[1];
  vertex_color.b = color[2];
  vertex_color.a = color[3];

  geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;

  Eigen::Vector3d v1(-sca[0] / 2, -sca[1] / 2, 0);
  Eigen::Vector3d v2(sca[0] / 2, -sca[1] / 2, 0);
  Eigen::Vector3d v3(sca[0] / 2, sca[1] / 2, 0);
  Eigen::Vector3d v4(-sca[0] / 2, sca[1] / 2, 0);
  Eigen::Vector3d v5(-sca[0] / 2, -sca[1] / 2, sca[2]);
  Eigen::Vector3d v6(sca[0] / 2, -sca[1] / 2, sca[2]);
  Eigen::Vector3d v7(sca[0] / 2, sca[1] / 2, sca[2]);
  Eigen::Vector3d v8(-sca[0] / 2, sca[1] / 2, sca[2]);

  v1 = pos + ori * v1;
  v2 = pos + ori * v2;
  v3 = pos + ori * v3;
  v4 = pos + ori * v4;
  v5 = pos + ori * v5;
  v6 = pos + ori * v6;
  v7 = pos + ori * v7;
  v8 = pos + ori * v8;

  p1.x = v1[0];
  p1.y = v1[1];
  p1.z = v1[2];

  p2.x = v2[0];
  p2.y = v2[1];
  p2.z = v2[2];

  p3.x = v3[0];
  p3.y = v3[1];
  p3.z = v3[2];

  p4.x = v4[0];
  p4.y = v4[1];
  p4.z = v4[2];

  p5.x = v5[0];
  p5.y = v5[1];
  p5.z = v5[2];

  p6.x = v6[0];
  p6.y = v6[1];
  p6.z = v6[2];

  p7.x = v7[0];
  p7.y = v7[1];
  p7.z = v7[2];

  p8.x = v8[0];
  p8.y = v8[1];
  p8.z = v8[2];

  // Bottom
  vertices.push_back(p1);
  vertices.push_back(p2);
  vertices.push_back(p3);

  vertices.push_back(p3);
  vertices.push_back(p4);
  vertices.push_back(p1);

  // Top
  vertices.push_back(p5);
  vertices.push_back(p6);
  vertices.push_back(p7);

  vertices.push_back(p7);
  vertices.push_back(p8);
  vertices.push_back(p5);

  // Front
  vertices.push_back(p1);
  vertices.push_back(p2);
  vertices.push_back(p5);

  vertices.push_back(p2);
  vertices.push_back(p5);
  vertices.push_back(p6);

  // Back
  vertices.push_back(p3);
  vertices.push_back(p4);
  vertices.push_back(p8);

  vertices.push_back(p3);
  vertices.push_back(p7);
  vertices.push_back(p8);

  // Right
  vertices.push_back(p2);
  vertices.push_back(p3);
  vertices.push_back(p7);

  vertices.push_back(p2);
  vertices.push_back(p6);
  vertices.push_back(p7);

  // Left
  vertices.push_back(p1);
  vertices.push_back(p4);
  vertices.push_back(p5);

  vertices.push_back(p4);
  vertices.push_back(p5);
  vertices.push_back(p8);

  triangle_count += 12;

  for (int i = 0; i < 36; i++) {
    vertex_colors.push_back(vertex_color);
  }
}
//}

/* clear */  //{
void BatchVisualizer::clear() {
  vertices.clear();
  vertex_colors.clear();
  triangle_count = 0;
}
//}

/* publish */  //{
void BatchVisualizer::publish() {
  visualization_msgs::Marker msg;
  msg.header.frame_id    = frame;
  msg.header.stamp       = ros::Time::now();
  msg.ns                 = "current_namespace";
  msg.action             = visualization_msgs::Marker::ADD;
  msg.type               = visualization_msgs::Marker::TRIANGLE_LIST;
  msg.id                 = 666;
  msg.pose.position.x    = 0.0;
  msg.pose.position.y    = 0.0;
  msg.pose.position.z    = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = 1.0;
  msg.scale.y            = 1.0;
  msg.scale.z            = 1.0;
  msg.points             = vertices;
  msg.colors             = vertex_colors;
  /* std::cout << "Visualizing " << triangle_count << " triangles, " << vertices.size() << " points\n"; */

  visual_pub.publish(msg);
}
//}

//}

#endif /* BATCH_VISUALIZER_H */
