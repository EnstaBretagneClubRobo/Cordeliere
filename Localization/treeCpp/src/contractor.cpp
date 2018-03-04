#include "contractor.h"



using namespace std;
using namespace ibex;
using region = std::vector< std::pair<int, int> >;

Contractor::Contractor(Tree* pTree, vector<int> dims)
{
	this->pTree = pTree;
	// creation of the initial region 
    for (unsigned int i = 0; i < dims.size(); i++)
    {
        this->initialRegion.push_back(make_pair(0, dims[i]-1));
    }
}


ibex::IntervalVector Contractor::contract(ibex::IntervalVector box)
{
	vector<ibex::IntervalVector> set;
	deepFirstSearch(box, this->initialRegion, this->pTree->root, set);
	IntervalVector contractedBox = set[0];
	
	while(set.size() > 0)
	{
		contractedBox = contractedBox | set[0];
		set.erase(set.begin());
	}

	contractedBox = contractedBox & box;

	return contractedBox;
}

void deepFirstSearch(ibex::IntervalVector box, region currentRegion, Node& currentNode, vector<ibex::IntervalVector>& set)
{
	// cast the currentRegion from region to IntervalVector
	int ndims = currentRegion.size();
	IntervalVector currentInterval(ndims);
	for (int i = 0; i < ndims; i++)
	{
		currentInterval[i] = Interval(currentRegion[i].first, currentRegion[i].second+1);
	}

	// check if the region intersect the box
	if (box.subvector(0, box.size() - 2).intersects(currentInterval))
	{
		Interval measurement = box[box.size() - 1];
		Interval nodeValue = currentNode.getItv();
		IntervalVector nodeValueVect(1, nodeValue);
		if (nodeValue.is_subset(measurement))
		{
			set.push_back(cart_prod(currentInterval, nodeValueVect));
		}
		else if (nodeValue.intersects(measurement))
		{
			if (currentNode.getLeft() == nullptr)
			{
				set.push_back(cart_prod(currentInterval, nodeValueVect));
			}
			else
			{
				pair<region, region> childrenRegions = bissect(currentRegion, currentNode.getAxis());
				deepFirstSearch(box, childrenRegions.first, *(currentNode.getLeft()), set);
				deepFirstSearch(box, childrenRegions.second, *(currentNode.getRight()), set);
			}
		}
	}
	else
	{
		return;
	}
}