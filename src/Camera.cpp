#include "myslam/Camera.h"
namespace myslam {
  
Camera::Camera(){
}

Vector3d Camera::world2camera(const Vector3d& pw, const SE3& Tcw)
{
  return Tcw*pw;
}
  
Vector3d Camera::camera2world(const Vector3d& pc, const SE3& Tcw)
{
  return Tcw.inverse()*pc;
}

Vector2d Camera::camera2pixel(const Vector3d& pc)
{
  return Vector2d(fx_*pc(0,0)/pc(0,2)+cx_, fy_*pc(0,1)/pc(0,2)+cy_);
}

Vector2d Camera::world2pixel(const Vector3d& pw, const SE3& Tcw)
{
  return camera2pixel(world2camera(pw, Tcw));
}

Vector3d Camera::pixel2camera(const Vector2d& pp, double depth)
{
  return Vector3d(
    (pp(0,0)-cx_)*depth/fx_,
    (pp(0,1)-cy_)*depth/fy_,
    depth
  );
}

Vector3d Camera::pixel2world(const Vector2d& pp, const SE3& Tcw, double depth)
{
  return camera2world(pixel2camera(pp, depth), Tcw);
}
}