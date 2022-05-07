#include <string>
#include "kdtree.h"
#pragma once

template <typename PointT>

class PtsClustering
{
public:
	PtsClustering() {}

	void clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol);
	std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minClusterSize, int maxClusterSize);
};

template <typename PointT>
void PtsClustering<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol)
{
	processed[indice] = true;

	cluster.push_back(indice);
	std::vector<int> nearest = tree->search(cloud->points[indice], distanceTol);

	for (int id : nearest)
	{
		if (!processed[id])
		{
			clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
		}
	}
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> PtsClustering<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minClusterSize, int maxClusterSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	KdTree *tree = new KdTree;

	for (int i = 0; i < cloud->points.size(); i++)
		tree->insert(cloud->points[i], i);

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processed;
	processed.assign(cloud->points.size(), false);

	int j = 0;
	while (j < cloud->points.size())
	{
		if (processed[j])
		{
			j++;
			continue;
		}
		std::vector<int> cluster_indices;
		typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
		clusterHelper(j, cloud, cluster_indices, processed, tree, distanceTol);

		int cluster_size = cluster_indices.size();

		if (cluster_size >= minClusterSize && cluster_size <= maxClusterSize)
		{
			for (int k = 0; k < cluster_size; k++)
			{
				cloudCluster->points.push_back(cloud->points[cluster_indices[k]]);
			}

			cloudCluster->width = cloudCluster->points.size();
			cloudCluster->height = 1;

			clusters.push_back(cloudCluster);
		}
	}

	return clusters;
}