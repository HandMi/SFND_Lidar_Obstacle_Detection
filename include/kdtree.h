#ifndef KDTREE_H_
#define KDTREE_H_

#include <vector>

template <typename PointT> struct Node
{
    PointT point;
    int id;
    Node *left;
    Node *right;

    Node(PointT arr, int setId) : point(arr), id(setId), left(NULL), right(NULL)
    {
    }
};

template <typename PointT> struct KdTree
{
    using NodeT = Node<PointT>;
    NodeT *root;

    KdTree() : root(NULL) {}

    void insertNode(NodeT *&node, std::size_t depth, PointT point, int id)
    {
        if (!node)
        {
            node = new NodeT(point, id);
        }
        else
        {
            auto dim = depth % 3;
            int l, r;
            if (dim == 0)
            {
                l = point.x;
                r = node->point.x;
            }
            else if (dim == 1)
            {
                l = point.y;
                r = node->point.y;
            }
            else
            {
                l = point.z;
                r = node->point.z;
            }

            if (l < r)
            {
                insertNode(node->left, depth + 1, point, id);
            }
            else
            {
                insertNode(node->right, depth + 1, point, id);
            }
        }
    }

    void insert(PointT point, int id)
    {
        // Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the
        // root
        insertNode(root, 0, point, id);
    }

    void searchNodes(PointT target, NodeT *node, int depth, float distanceTol,
                     std::vector<int> &ids)
    {
        if (node)
        {
            if ((node->point.x >= (target.x - distanceTol) &&
                 node->point.x <= (target.x + distanceTol)) &&
                (node->point.y >= (target.y - distanceTol) &&
                 node->point.y <= (target.y + distanceTol)) &&
                (node->point.z >= (target.z - distanceTol) &&
                 node->point.z <= (target.z + distanceTol)))
            {
                auto distance = sqrt(
                    (node->point.x - target.x) * (node->point.x - target.x) +
                    (node->point.y - target.y) * (node->point.y - target.y) +
                    (node->point.z - target.z) * (node->point.z - target.z));
                if (distance <= distanceTol)
                {
                    ids.push_back(node->id);
                }
            }

            int dim = depth % 3;
            int l, r;
            if (dim == 0)
            {
                l = target.x;
                r = node->point.x;
            }
            else if (dim == 1)
            {
                l = target.y;
                r = node->point.y;
            }
            else
            {
                l = target.z;
                r = node->point.z;
            }

            // check across boundary
            if ((l - distanceTol) < r)
            {
                searchNodes(target, node->left, depth + 1, distanceTol, ids);
            }
            if ((l + distanceTol) > r)
            {
                searchNodes(target, node->right, depth + 1, distanceTol, ids);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchNodes(target, root, 0, distanceTol, ids);

        return ids;
    }
};

#endif /* KDTREE_H_ */