#include "ibex.h"

using region = std::vector< std::pair<int, int> >;

class Node
{

    public:
    Node();
    ~Node();
    void createBranch(std::vector< std::pair<region, Node*> > &leaves, region currentRegion);
    void fillNode();


    ibex::Interval getItv();
    void setItv(ibex::Interval interval);

    
    private:
    ibex::Interval itv;
    short axis;
    Node* left;
    Node* right;
};

std::pair<region, region> bissect(region rgn, int axis);
