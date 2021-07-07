#ifndef RANSAC_H_
#define RANSAC_H_

#include <pcl/common/common.h>
#include <unordered_set>

template <typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                    int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;

    // For max iterations

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers

    while (maxIterations > 0)
    {
        --maxIterations;
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
        {
            inliers.insert(rand() % (cloud->points.size()));
        }
        auto it = inliers.begin();
        auto x1 = cloud->points[*it].x;
        auto y1 = cloud->points[*it].y;
        auto z1 = cloud->points[*it].z;
        ++it;
        auto x2 = cloud->points[*it].x;
        auto y2 = cloud->points[*it].y;
        auto z2 = cloud->points[*it].z;
        ++it;
        auto x3 = cloud->points[*it].x;
        auto y3 = cloud->points[*it].y;
        auto z3 = cloud->points[*it].z;

        auto a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        auto b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        auto c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        auto d = -(a * x1 + b * y1 + c * z1);

        for (int i = 0; i < cloud->points.size(); ++i)
        {
            if (inliers.count(i) > 0)
            {
                continue;
            }

            PointT point = cloud->points[i];
            auto x = point.x;
            auto y = point.y;
            auto z = point.z;

            auto dist =
                fabs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c);

            if (dist <= distanceTol)
            {
                inliers.insert(i);
            }
            if (inliers.size() > inliersResult.size())
            {
                inliersResult = inliers;
            }
        }
    }

    return inliersResult;
};

#endif /* RANSAC_H_ */