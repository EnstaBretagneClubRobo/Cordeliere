#include "node.h"

using namespace std;
using region = std::vector< std::pair<int, int> >;

Node::Node()
{
    
}


void Node::createBranch(vector< pair<region, Node*> > &leaves, region currentRegion)
{
    // check if the region corresponds to a leaf (i.e. currentRegion = pixel)
    bool isALeaf = true;
    for (unsigned int i = 0; i < currentRegion.size(); i++)
    {
        if (currentRegion[i].first != currentRegion[i].second)
        {
            isALeaf = false;
        }
    }

    cout << "isALeaf test end" << endl;

    if (isALeaf)
    {
        pair<region, Node*> leaf;
        leaf.first = currentRegion;
        leaf.second = this;
    }
    else
    {
        // creation of the children nodes
        Node leftNode, rightNode;
        this->left = &leftNode;
        this->right = &rightNode;

        // currentRegion bissection
        int maxDimLength = 0;
        for (unsigned int i = 0; i < currentRegion.size(); i++)
        {
            int dimLength = currentRegion[i].second - currentRegion[i].first;
            if (dimLength > maxDimLength)
            {
                this->axis = i;
                maxDimLength = dimLength;
            }
        }
        pair<region, region> childrenRegions = bissect(currentRegion, this->axis);

        // creation of the children branchings
        this->left->createBranch(leaves, childrenRegions.first);
        this->right->createBranch(leaves, childrenRegions.second);
    }

}

pair<region, region> bissect(region rgn, int axis)
{
    int length = rgn[axis].second - rgn[axis].first + 1;

    region leftRegion = rgn;
    region rightRegion = rgn;

    leftRegion[axis].second = rgn[axis].first + length/2 - 1;
    rightRegion[axis].first = rgn[axis].first + length/2;

    pair<region, region> childrenRegions = {leftRegion, rightRegion};
    return childrenRegions;
}