#include <vector>
#include "tree.h"

using region = std::vector< std::pair<int, int> >;

class Contractor
{
	public:
		Contractor(Tree* pTree, region initialRegion);
		ibex::IntervalVector contract(ibex::IntervalVector box);

	private:
		Tree* pTree;
		region initialRegion;
};

void deepFirstSearch(ibex::IntervalVector box, region currentRegion, Node& currentNode, std::vector <ibex::IntervalVector>& set);
