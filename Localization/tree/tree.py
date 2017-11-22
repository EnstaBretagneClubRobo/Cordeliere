import anytree
from anytree.dotexport import RenderTreeGraph
import numpy as np
import pyibex as pyibex


class Tree:
    def __init__(self, data, ndim):
        self.root = anytree.Node('root', itv=None)
        self.fill(data, ndim)

    def fill(self, data, ndim):
        self.root.itv = pyibex.Interval(data.min(), data.max())




if __name__ == "__main__":
    root = anytree.Node('root', itv=0)
    # > dot file.dot -Tpng -o tree.png (needs graphviz installed)
    RenderTreeGraph(root).to_dotfile("tree.dot")

    data = np.array([[1,2,3],[4,5,6]])
    tree = Tree(data,2)
    print(tree.root.itv)