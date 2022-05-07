#include <unordered_set>

#include <pcl/common/common.h>

template <typename PointT>
class ransac3d
{

public:
  ransac3d() {}
  std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
};

/**********************************************************************/

// implementing the ransac algorithm
template <typename PointT>
std::unordered_set<int> ransac3d<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;

  auto cloud_points = cloud->points;

  // walk through all points
  for (int i = 0; i < maxIterations; i++)
  {
    std::unordered_set<int> inliers;
    //getting the 3 different random points to start with
    while (inliers.size() < 3)
    {
      inliers.insert(rand() % (cloud->points.size()));
    }

    // extract 3-points to define plane
    float x1, x2, x3, y1, y2, y3, z1, z2, z3;
    auto itr = inliers.begin();
    x1 = cloud_points[*itr].x;
    y1 = cloud_points[*itr].y;
    z1 = cloud_points[*itr].z;
    itr++;
    x2 = cloud_points[*itr].x;
    y2 = cloud_points[*itr].y;
    z2 = cloud_points[*itr].z;
    itr++;
    x3 = cloud_points[*itr].x;
    y3 = cloud_points[*itr].y;
    z3 = cloud_points[*itr].z;

    // define plane equation coefficients
    float a, b, c, d;
    a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    d = -(a * x1 + b * y1 + c * z1);

    // implement RANSAC via point-to-plane distance check
    for (int j = 0; j < cloud->points.size(); j++)
    {

      if (inliers.count(j) > 0)
        continue;

      float x, y, z, dist;

      x = cloud_points[j].x;
      y = cloud_points[j].y;
      z = cloud_points[j].z;

      dist = fabs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c);

      if (dist <= distanceTol)
      {
        inliers.insert(j);
      }

      if (inliers.size() > inliersResult.size())
      {
        inliersResult = inliers;
      }
    }
  }

  return inliersResult;
}