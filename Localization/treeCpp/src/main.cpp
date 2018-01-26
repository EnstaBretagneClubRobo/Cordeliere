#include "tree.h"
#include <vector>

using namespace std;

int main()
{
    vector<int> dim;    
        
    Tree tree;
    tree.fill(vector<int> {2,4}, vector<float> {1, 2, 3, 4, 5, 6, 8, 6});

    // cout << sub2ind(vector<int> {4, 6, 3}, vector<int> {2, 2, 2}) << endl;
    // cout << sub2ind(vector<int> {3, 4, 2}, vector<int> {2, 2, 1}) << endl;

    return 0;
}
