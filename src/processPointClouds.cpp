// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include "cluster.cpp"
// Function Name: Constructor for the class "ProcessPointClouds"
// Function Description: This class contains all methods that will be used to process the lidar pointcloud
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// Function Name: de-constructor for the class "ProcessPointClouds"
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

// Function Name: numPoints
// Function Description: Prints the number of points available in the pointcloud
// Inputs:
// - cloud: pointCloud to be processed
// Outputs: Non
template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
        std::cout << cloud->points.size() << std::endl;
}

// Function Name: FilterCloud
// Function Description: - Filter the point cloud data
// - Downsample the data for faster processing
// - According to the region of interest
//                       - Compute a time cost of the function
// Inputs:
// - cloud: pointCloud to be processed
// - filterRes: the resolution of the data that will be processed after that
// Outputs: Non
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();

        // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
        pcl::VoxelGrid<PointT> voxel;
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

        voxel.setInputCloud(cloud);
        voxel.setLeafSize(filterRes, filterRes, filterRes);
        voxel.filter(*cloud_filtered);

        typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

        pcl::CropBox<PointT> region(true);
        region.setMin(minPoint);
        region.setMax(maxPoint);
        region.setInputCloud(cloud_filtered);
        region.filter(*cloudRegion);

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

        return cloudRegion;
}

// Function Name: SeparateClouds
// Function Description: Create two new point clouds, one cloud with obstacles and other with segmented plane
// Inputs:
// - inliers:
// - cloud: pointCloud to be processed
// Outputs:
// - segResult:
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
        // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
        typename pcl::PointCloud<PointT>::Ptr obstaclecloud(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr planecloud(new pcl::PointCloud<PointT>());

        for (int index : inliers->indices)
        {
                planecloud->points.push_back(cloud->points[index]);
        }
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*obstaclecloud);

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclecloud, planecloud);
        return segResult;
}

//////////////////////////////////3D Ransac/////////////////////////////////////////////
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentRansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
        auto startTime = std::chrono::steady_clock::now();
        srand(time(NULL));

        int num_points = cloud->points.size();
        auto cloud_points = cloud->points;

        ransac3d<PointT> seg;

        // get inliers from local-RANSAC implementation rather than PCL implementation
        std::unordered_set<int> inliersResult = seg.Ransac(cloud, maxIterations, distanceTol);

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "RANSAC 3D plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        typename pcl::PointCloud<PointT>::Ptr out_plane(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr in_plane(new pcl::PointCloud<PointT>());

        for (int i = 0; i < cloud->points.size(); i++)
        {
                PointT pt = cloud_points[i];
                if (inliersResult.count(i))
                {
                        out_plane->points.push_back(pt);
                }
                else
                {
                        in_plane->points.push_back(pt);
                }
        }

        return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(in_plane, out_plane);
}
/////////////////////////////end 3D Ransac//////////////////////////////////////////////////

// Function Name: SegmentPlane
// Function Description: Do plane Segmentatio to detect the road carpet using RANSAC algorithm
// Inputs:
// - cloud: pointCloud to be processed
// - maxIterations: Maximum number of iletrations for the algorithm
// - distanceThreshold: Maximum distance, a way from the trial plane, for a point to be added to the cluster
// Outputs:
// - segResult:
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();
        // TODO:: Fill in this function to find inliers for the cloud.
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distanceThreshold);
        seg.setMaxIterations(maxIterations);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
                std::cout << "Could not estimate a planar model for the given dataset.\n";
        }
        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_kdtree(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

        // Time clustering process
        auto startTime = std::chrono::steady_clock::now();

        PtsClustering<PointT> clusterer;
        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = clusterer.euclideanCluster(cloud, clusterTolerance, minSize, maxSize);

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering with kdtree took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

        return clusters;
}

// Function Name: Clustering
// Function Description: Do Ecludean clustering for detecting objects
// Inputs:
// - cloud: pointCloud to be processed
// - clusterTolerance: Maximum redius of the cluster
// - minSize: Minimum no. of points to be considered as a cluster/object
// - maxSize: Maximum no. of points to be considered as a cluster/object
// Outputs:
// - clusters: Vector of pointclouds pointers. i.e vector of pointers to objects' first points
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

        // Time clustering process
        auto startTime = std::chrono::steady_clock::now();

        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

        // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
        // Creating the KdTree object for the search method of the extraction
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(clusterTolerance); // 2cm
        ec.setMinClusterSize(minSize);
        ec.setMaxClusterSize(maxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (pcl::PointIndices getIndices : cluster_indices)
        {
                typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
                for (int index : getIndices.indices)
                        cloud_cluster->points.push_back(cloud->points[index]); //*
                cloud_cluster->width = cloud_cluster->points.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                clusters.push_back(cloud_cluster);
        }
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

        return clusters;
}

// Function Name: BoundingBox
// Function Description: Plotting 3D Boundary box around the detected cluster/object
// Inputs:
// - cluster: group of points the belongto the same object
// Outputs:
// - box: 3d Bounding box (structure of [min, max][x;y;z])
template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

        // Find bounding box for one of the clusters
        PointT minPoint, maxPoint;
        pcl::getMinMax3D(*cluster, minPoint, maxPoint);

        Box box;
        box.x_min = minPoint.x;
        box.y_min = minPoint.y;
        box.z_min = minPoint.z;
        box.x_max = maxPoint.x;
        box.y_max = maxPoint.y;
        box.z_max = maxPoint.z;

        return box;
}

// Function Name: savePcd
// Function Description: save pointcloud to a certain path
// Inputs:
// - cloud: pointCloud to be processed
// - file: output file path
// Outputs: Non
template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
        pcl::io::savePCDFileASCII(file, *cloud);
        std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

// Function Name: loadPcd
// Function Description: load pointcloud from a certain path
// Inputs:
// - file: input file path
// Outputs:
// - cloud: loaded pointCloud
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

        if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
        {
                PCL_ERROR("Couldn't read file \n");
        }
        std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

        return cloud;
}

// Function Name: streamPcd
// Function Description: sort files that are in a certain directory in accending order so playback is chronological
// Inputs:
// - dataPath: input directory path
// Outputs:
// - paths: vector containing sorted paths of point cloud data in this directory
template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

        std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

        // sort files in accending order so playback is chronological
        sort(paths.begin(), paths.end());

        return paths;
}