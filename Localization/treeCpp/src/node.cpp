#include "node.h"

using namespace std;
using region = std::vector< std::pair<int, int> >;

Node::Node()
{
    this->p_isALeaf = &isALeaf; 
    ibex::Interval itv_init(0,0);
    this->setItv(itv_init);
    this->right = nullptr;
    this->left = nullptr;
}

Node::~Node()
{
    delete this->right;
    delete this->left;
}



void Node::createBranch(vector< pair<region, Node*> > &leaves, region currentRegion)
{
    cout << "test createBranch" << endl;
    // check if the region corresponds to a leaf (i.e. currentRegion = pixel)
    this->isALeaf = true;
    for (unsigned int i = 0; i < currentRegion.size(); i++)
    {
        if (currentRegion[i].first != currentRegion[i].second)
        {
            this->isALeaf = false;
            break;
        }
    }


    if (this->isALeaf)
    {
        pair<region, Node*> leaf;
        leaf.first = currentRegion;
        leaf.second = this;
        leaves.push_back(leaf);
    }
    else
    {
        // creation of the children nodes
        Node* leftNode = new Node();
        Node* rightNode = new Node();
        this->left = leftNode;
        this->right = rightNode;

        // currentRegion bissection (along the first max length dimension : [1 3 3]                                                                     ^
        int maxDimLength = 0;  //                                              ^
        for (unsigned int i = 0; i < currentRegion.size(); i++)
        {
            int dimLength = currentRegion[i].second - currentRegion[i].first + 1;
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

ibex::Interval Node::getItv()
{
    return this->itv;
}

void Node::setItv(ibex::Interval interval)
{
    this->itv = interval;
}

void Node::fillNode()
{

    //if (this->right->right != NULL)
    if (!this->right->isALeaf)
    {
        this->right->fillNode();
    }
    //if (this->left->right != NULL)
    if (!this->left->isALeaf)
    {
        this->left->fillNode();
    }
    cout << "test fillNode" << endl;
    this->itv = this->right->itv|this->left->itv;
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

