#include "ibex.h"

using region = std::vector< std::pair<int, int> >;

class Node
{

    public:
    Node();
    void createBranch(std::vector< std::pair<region, Node*> > &leaves, region currentRegion);
    Node* left;
    Node* right;
    bool isALeaf;
    bool* p_isALeaf;


    ibex::Interval getItv();

    
    private:
    ibex::Interval itv;
    short axis;
    //Node* left;
    //Node* right;
};

std::pair<region, region> bissect(region rgn, int axis);
