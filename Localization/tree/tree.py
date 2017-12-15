############################################

# Tree class to perform a contraction with

############################################

import numpy as np
import pyibex as pyibex


class Node:
    """Node class

    A Node is an object that contains the range (=interval) of
    the physical value in the corresponding region of the data set.
    It also contains the axis along which the region has been bisected.

    Variables:
        itv   {pyibex.Interval} -- the range of the physical value in the corresponding region
        axis  {integer}         -- the axis along which the region has been bisected
        left  {Node}            -- the Node corresponding to one of the two regions obtained
                                   after the bisection of the region
        right {Node}            -- //
    """

    def __init__(self, itv=None, axis=None, left=None, right=None):
        self.itv = itv
        self.axis = axis
        self.left = left
        self.right = right

    def visitDisplay(self):
        """The method that builds the content of the .dot file used to draw the tree.

        (see Tree.saveAsDot method)

        Returns:
            dotStr {string} -- a segment of the content of the .dot file
        """

        # add a node on the tree identified by the Python id.
        dotStr = str(id(self)) + " [label=\"{0}\"];\n".format(self.itv)

        # left child
        if self.left is not None:
            dotStr += self.left.visitDisplay()  # visit the left child of the node
            dotStr += str(id(self)) \
                + " -> " \
                + str(id(self.left)) \
                + " [label=\"{0}\"];\n".format(self.axis)  # add a link between the node and its left child

        # right child
        if self.right is not None:
            dotStr += self.right.visitDisplay()  # visit the right child of the node
            dotStr += str(id(self)) \
                + " -> " \
                + str(id(self.right)) \
                + " [label=\"{0}\"];\n".format(
                self.axis)  # add a link between the node and its right child
        return dotStr

    def visitContract(self, testedBox, nodeBox):
        if nodeBox.is_subset(testedBox):
            return self.itv
        
        nodeBoxLeft, nodeBoxRight = nodeBox.bisect(node.axis, ratio=np.ceil(nodeBox[node.axis].diam()/2)/nodeBox[node.axis].diam())
        if nodeBoxLeft.intersects(testedBox):

            
            # nodeBoxLeft = pyibex.IntervalVector([[round(nodeBoxLeft[0][0]), round(nodeBoxLeft[0][1])],
            #                                      [round(nodeBoxLeft[1][0]),round(nodeBoxLeft[1][1])]])
            # nodeBoxRight = pyibex.IntervalVector([[round(nodeBoxRight[0][0]), round(nodeBoxRight[0][1])],
            #                                      [round(nodeBoxRight[1][0]),round(nodeBoxRight[1][1])]])
            self.left.visitContract(testBox, nodeBoxLeft)
            self.right.visitContract(testBox, nodeBoxRight)

    
    


class Tree:
    """Tree class

    An object of this class contains the different ranges (=interval) of the physical value of a dataset
    in different regions of the dataset.
    The different regions are obtained using multiple bisections of the dataset.

    Variables:
        data {numpy.array} -- the dataset
        root {Node}        -- the root node. It therefore contains the range of the physical value in the whole dataset
    """

    def __init__(self, data):
        self.root = Node()
        self.fillNode(self.root, data)  # fill the tree

    def fillNode(self, node, data):
        """Fill a node with a subdataset 

        This method is called recursively :
        a node is filled when all its children nodes are filled.

        Arguments:
            node {Node}        -- the node to fill
            data {numpy.array} -- the subdataset to fill the node with
        """

        # the dataset is reduced to a cell. It can't be bisected
        if max(data.shape) == 1:
            node.itv = pyibex.Interval(data[0, 0], data[0, 0])

        # the dataset can still be bisected
        else:
            # the bisection is performed along the larger side
            node.axis = np.argmax(data.shape)
            node.left = Node()
            node.right = Node()
            # fill both children with a half of the dataset each
            self.fillNodeChildren(node,
                                  np.array_split(data, 2, axis=node.axis))
            # compute the interval (ex: left=[4,6], right=[5,7] ==> node=[4,7])
            node.itv = node.left.itv | node.right.itv

    def fillNodeChildren(self, node, data_list):
        """Fill the children of a node with two halves of a dataset

        Arguments:
            node      {Node} -- the node whose children will be filled
            data_list {[list of two numpy.array} -- the two halves of the dataset
        """
        self.fillNode(node.left, data_list[0])
        self.fillNode(node.right, data_list[1])

    def saveAsDot(self, filename):
        """save a .dot file of the tree

        ex of use: dot tree.dot -Tpng -o tree.png

        Arguments:
            filename {string} -- the output file name (ex: tree.dot)
        """
        dotStr = "digraph tree {\n"
        dotStr += self.root.visitDisplay()  # generate the content of the file
        dotStr += "}"

        # save
        with open(filename, 'w') as file:
            file.write(dotStr)


if __name__ == "__main__":

    # data = np.array([[1,2,3, 4, 5, 6, 7],
                     # [8,9,10,11,12,13,14]])
    data = np.array([[[1, 2], [3, 4]], [[5, 6], [7, 8]]])
    tree = Tree(data)
    tree.saveAsDot("tree.dot")
