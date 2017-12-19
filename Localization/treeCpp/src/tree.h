#include "node.h"
#include <vector>

class Tree
{   
    public:
    Tree();
    void fillNode(Node node);
    void fill(std::vector<int> dims, std::vector<float> data);
    
    private:
    Node root;
    
};
