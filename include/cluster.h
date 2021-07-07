#ifndef CLUSTER_H_
#define CLUSTER_H_

#include "kdtree.h"

template <typename PointT>
void proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud,
               typename pcl::PointCloud<PointT>::Ptr &cluster,
               std::vector<bool> &processed, KdTree<PointT> *tree,
               float distanceTol)
{
    processed[index] = true;
    cluster->points.push_back(cloud->points[index]);
    std::vector<int> nearest = tree->search(cloud->points[index], distanceTol);

    for (auto id : nearest)
    {
        if (!processed[id])
        {
            proximity(id, cloud, cluster, processed, tree, distanceTol);
        }
    }
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol,
                 int minSize, int maxSize)
{
    const auto cloudSize = cloud->points.size();
    auto *tree = new KdTree<PointT>;

    for (int i = 0; i < cloudSize; ++i)
    {
        tree->insert(cloud->points[i], i);
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    std::vector<bool> processed(cloudSize, false);

    int i = 0;
    while (i < cloudSize)
    {
        if (processed[i])
        {
            ++i;
            continue;
        }
        typename pcl::PointCloud<PointT>::Ptr cluster(
            new pcl::PointCloud<PointT>);
        proximity(i, cloud, cluster, processed, tree, distanceTol);
        if (cluster->points.size() >= minSize &&
            cluster->points.size() <= maxSize)
        {
            clusters.push_back(cluster);
        }
        ++i;
    }

    return clusters;
}

#endif