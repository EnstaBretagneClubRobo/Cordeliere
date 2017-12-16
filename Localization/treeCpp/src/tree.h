#include "node.h"
#include <vector>

class Tree
{   
    public:
    Tree(std::vector<int> dims, std::vector<float> data);
    void fillNode(Node node);
    
    private:
    Node root();
    
};
