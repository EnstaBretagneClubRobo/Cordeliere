#include "node.h"
#include <vector>

using namespace std;
using region = std::vector< std::pair<int, int> >;

class Tree
{   
    public:
    Tree();
    void fill(vector<int> dims, vector<float> data);
    
    private:
    Node root;
    void fillNode(Node node);
    void fillLeaves(vector< pair<region, Node*> > leaves, vector<float> data);
    
};
