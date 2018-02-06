#include "node.h"
#include <vector>
#include <algorithm>

using region = std::vector< std::pair<int, int> >;

class Tree
{   
    public:
    Node root;
    Tree();
    ~Tree();
    void fill(std::vector<int> dims, std::vector<float> data);
    void DFS(Node* head);
    
    private:
    //Node root;
    void fillNode(Node node);
    void fillLeaves(std::vector< std::pair<region, Node*> > &leaves, std::vector<float> data, std::vector<int> dims);
    
};

int sub2ind(std::vector<int> dims, std::vector<int> sub);

