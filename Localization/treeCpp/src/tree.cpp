#include "tree.h"

using namespace std;
using region = std::vector< std::pair<int, int> >;

Tree::Tree()
{
    Node root();         
}




void Tree::fillNode(Node node)
{

}


void Tree::fillLeaves(vector< pair<region, Node*> > &leaves, vector<float> data, vector<int> dims)
{
    cout << "test fillLeaves" << endl;
    for (unsigned int i = 0 ; i < leaves.size(); i++)
    {
        pair<region, Node*> leaf = leaves[i];

        //get the coordinate (pixel) associated with the Node
        vector<int> pixel;
        for (unsigned int j = 0; j < leaf.first.size(); j++)
        {
            pixel.push_back(leaf.first[j].first);
        }

        //get the value of the pixel
        float val = data[sub2ind(dims, pixel)];

        float v_min =val;
        float v_max = val;
        for (unsigned int j = 0; j < leaf.first.size(); j++)
        {
            for (int k = -1; k <= 1; k += 2)
            {
                //get the value of the neighbour pixel
                vector<int> neighbour_pixel = pixel;
                neighbour_pixel[j] += k;

                //check if coordinate is not out of bound
                bool pixelInData = true;
                for (unsigned int l =0; l < dims.size(); l++)
                {
                    if (neighbour_pixel[l] < 0 or neighbour_pixel[l] >= dims[l])
                    {
                        pixelInData = false;
                        break;
                    }
                }
                if (pixelInData)
                {
                    //get the value in the neighbour pixel
                    float nval = data[sub2ind(dims, neighbour_pixel)];

                    //get the min and max
                    v_min = min(v_min, nval);
                    
                    v_max = max(v_max, nval);
                }
            }
        
        }
    ibex::Interval result_itv(v_min, v_max);

    leaves[i].second->setItv(result_itv);

    }
}

void Tree::fill(vector<int> dims, vector<float> data)
{
    // creation of the vector 
    region currentRegion;
    for (unsigned int i = 0; i < dims.size(); i++)
    {
        currentRegion.push_back(make_pair(0, dims[i]-1));
    }

    // creation of the list of the leaves
    vector< pair<region, Node*> > leaves;

    // recursive creation of the tree branching
    this->root.createBranch(leaves, currentRegion);

    // fill the leaves
    fillLeaves(leaves, data, dims);

    // fill the rest of the tree
    this->root.fillNode();
    cout << "root " << this->root.getItv() << endl;

    for (unsigned int i = 0; i < leaves.size(); i++)
    {
        // cout << leaves[i].first[0].first << leaves[i].first[0].second << endl;
        // cout << leaves[i].first[1].first << leaves[i].first[1].second << endl << endl;
        cout << "[" << leaves[i].first[0].first << "," << leaves[i].first[1].first << "] -- " << leaves[i].second->getItv() << endl;
    }
}


int sub2ind(vector<int> dims, vector<int> sub)
//convert subscripts to linear indices
{
    int ind = 0;
    int a = 1;
    for (int i = dims.size() - 1; i >= 0; i--)
    {
        ind += a * sub[i];
        a *= dims[i];
    }
    return ind;
}


void Tree::DFS(Node* head)
//Depth First Search Traversal
{
    bool debug=true;
    if (debug){
        cout << "first_passage";
        debug=false;
    }
    if (not head->isALeaf)
    {
        if (not head->left->isALeaf)
        {
            DFS(head->left);
        }
        if (not head->right->isALeaf)
        {
            DFS(head->right);
        }
        cout << "Current interval:" << head->getItv() << endl;
        //printf("%d  ", head.itv);
    }
}
