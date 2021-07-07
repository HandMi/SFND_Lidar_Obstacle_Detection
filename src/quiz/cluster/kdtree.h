/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId)
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

    KdTree() : root(NULL) {}

    ~KdTree() { delete root; }

    void insertNode(Node *&node, std::size_t depth, std::vector<float> point,
                    int id)
    {
        if (!node)
        {
            node = new Node(point, id);
        }
        else
        {
            auto dim = depth % 2;
            if (point[dim] < (node->point[dim]))
            {
                insertNode(node->left, depth + 1, point, id);
            }
            else
            {
                insertNode(node->right, depth + 1, point, id);
            }
        }
    }

    void insert(std::vector<float> point, int id)
    {
        // Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the
        // root
        insertNode(root, 0, point, id);
    }

    void searchNodes(std::vector<float> target, Node *node, int depth,
                     float distanceTol, std::vector<int> &ids)
    {
        if (node)
        {
            if ((node->point[0] >= (target[0] - distanceTol) &&
                 node->point[0] <= (target[0] + distanceTol)) &&
                (node->point[1] >= (target[1] - distanceTol) &&
                 node->point[1] <= (target[1] + distanceTol)))
            {
                auto distance = sqrt((node->point[0] - target[0]) *
                                         (node->point[0] - target[0]) +
                                     (node->point[1] - target[1]) *
                                         (node->point[1] - target[1]));
                if (distance <= distanceTol)
                {
                    ids.push_back(node->id);
                }
            }

            // check across boundary
            if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
            {
                searchNodes(target, node->left, depth + 1, distanceTol, ids);
            }
            if ((target[depth % 2] + distanceTol) > node->point[depth % 2])
            {
                searchNodes(target, node->right, depth + 1, distanceTol, ids);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchNodes(target, root, 0, distanceTol, ids);

        return ids;
    }
};
