#include <unordered_set>

#include <pcl/common/common.h>

template<typename PointT>
class ransac3d{

public:
  ransac3d() {}
  std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
};
