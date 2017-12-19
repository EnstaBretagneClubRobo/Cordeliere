#include "ibex.h"


class Node
{

    public:
    Node();
    

    private:
    ibex::Interval itv;
    short axis;
    Node left();
    Node right();

};



