#include "node.h"
#include <vector>
#include <algorithm>

using region = std::vector< std::pair<int, int> >;

class Tree
{   
    public:
    Tree();
    void fill(std::vector<int> dims, std::vector<float> data);
    
    private:
    Node root;
    void fillNode(Node node);
    void fillLeaves(std::vector< std::pair<region, Node*> > leaves, std::vector<float> data, std::vector<int> dims);
    
};

int sub2ind(std::vector<int> dims, std::vector<int> sub);