#include "tree.h"
#include <vector>

using namespace std;

int main()
{
    vector<int> dim;    
        
    Tree tree;
    tree.fill(vector<int> {2,4}, vector<float> {1, 2, 3, 4, 5, 6, 8, 6});

    return 0;
}
