#include <pcl/visualization/pcl_visualizer.h>
#include "render/box.h"
#include <iostream>
#include <vector>
#include <string>
#pragma once

// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node *left;
	Node *right;

	Node(pcl::PointXYZI arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	~KdTree()
	{
		delete root;
	}

	void insertNode(Node **node, uint depth, pcl::PointXYZI point, int id)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint cd = depth % 3;
			if (cd == 0)
			{
				if (point.x < ((*node)->point.x))
				{
					insertNode(&((*node)->left), depth + 1, point, id);
				}
				else
				{
					insertNode(&((*node)->right), depth + 1, point, id);
				}
			}
			else if (cd == 1)
			{
				if (point.y < ((*node)->point.y))
				{
					insertNode(&((*node)->left), depth + 1, point, id);
				}
				else
				{
					insertNode(&((*node)->right), depth + 1, point, id);
				}
			}
			else
			{

				if (point.z < ((*node)->point.z))
				{
					insertNode(&((*node)->left), depth + 1, point, id);
				}
				else
				{
					insertNode(&((*node)->right), depth + 1, point, id);
				}
			}
		}
	}

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertNode(&root, 0, point, id);
	}

	void searchHelper(pcl::PointXYZI target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
	{

		if (node != NULL)
		{
			float delta_x = node->point.x - target.x;
			float delta_y = node->point.y - target.y;
			float delta_z = node->point.z - target.z;

			if ((-distanceTol <= delta_x && distanceTol >= delta_x) &&
				(-distanceTol <= delta_y && distanceTol >= delta_y) &&
				(-distanceTol <= delta_z && distanceTol >= delta_z))
			{
				float distance = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
				// check distance
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			if (depth % 3 == 0) // 3 dim kd-tree - x-axis
			{
				if (-distanceTol < delta_x)
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				if (distanceTol > delta_x)
					searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
			else if (depth % 3 == 1) // y-axis
			{
				if (-distanceTol < delta_y)
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				if (distanceTol > delta_y)
					searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
			else // z-axis
			{
				if (-distanceTol < delta_z)
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				if (distanceTol > delta_z)
					searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};
