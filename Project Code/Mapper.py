from Queue import PriorityQueue
import MapGraph


class Mapper:

    def __init__(self, graph, startVert, goalVert):
        self.graph = MapGraph.readMapFile(graph)
        self.startVert = startVert
        self.goalVert = goalVert
        self.maxVal = 1000
        self.U = None
        self.rhs = {}
        self.g = {}
        self.initialize()

    def initialize(self):
        """ Initializes the algorithm by setting costs to infinity
        and start costs to 0."""
        self.U = PriorityQueue()
        for s in self.graph.getVertices():
            self.rhs[s] = float('inf')
            self.g[s] = float('inf')
        self.rhs[self.startVert] = 0
        self.U.insert(self.startVert, self.calculateKey(self.startVert))

    def calculateKey(self, vert):
        """The CalculateKey algorithm from the pseudocode."""
        minG = min(self.g[vert], self.rhs[vert])
        heurCost = self.graph.heuristicDist(vert, self.goalVert)
        return (minG + heurCost, minG)

    def calculateKeys(self, key1, key2):
        """ Finds total cost by adding minimum g to heuristic cost. """
        [f1, g1] = key1
        [f2, g2] = key2
        return (f1 < f2) or ((f1 == f2) and (g1 < g2))

    def computeShortestPath(self):
        """ Finds the least costly path to the goal. """
        while self.U.getSize() != 0 and ((self.U.firstElement() < self.calculateKey(self.goalVert)) or
                                    (self.rhs[self.goalVert] != self.g[self.goalVert])):
            u = self.U.dequeue()
            if self.g[u[0]] > self.rhs[u[0]]:
                self.g[u[0]] = self.rhs[u[0]]
            else:
                self.g[u[0]] = float('inf')
                self.updateVertex(u)
            for s in self.graph.getNeighbors(u[0]):
                self.updateVertex(s[0])
        return self.U

    def updateVertex(self, vert):
        """ Updates the priority queue based on key. """
        if vert != self.startVert:
            self.rhs[vert] = self.minNeighCost(vert)
        if self.U.contains(vert):
            self.U.removeValue(vert)
        if self.g[vert] != self.rhs[vert]:
            self.U.insert(vert, self.calculateKey(vert))

    def minNeighCost(self, vert):
        """A helper to compute the new rhs value, by finding the minimum cost among
        all the neighbors of a vertex. The cost is computed as the g cost of the
        neighbor plus the edge cost between the neighbor and the vertex."""
        minNCost = self.maxVal
        minVert = -1
        for neighInfo in self.graph.getNeighbors(vert):
            neigh = neighInfo[0]
            edgeCost = neighInfo[1]
            newCost = self.g[neigh] + edgeCost
            if newCost < minNCost:
                minNCost = newCost
                minVert = neigh
        return minNCost

    def _compareKeys(self, key1, key2):
        """Takes in two keys, each of which is a list containing f cost
        and g cost. It prefers the lower f cost, but for equal f costs
        it chooses the lower g cost."""
        [f1, g1] = key1
        [f2, g2] = key2
        return (f1 < f2) or ((f1 == f2) and (g1 < g2))

    def _pickMinNeighbor(self, vert):
        """A helper to path-reconstruction that finds the neighbor of a vertex
        that has the minimum g cost."""
        neighs = self.graph.getNeighbors(vert)
        minVal = self.maxVal
        minNeigh = None
        for [neigh, cost] in neighs:
            if self.g[neigh] < minVal:
                minVal = self.g[neigh]
                minNeigh = neigh
        return minNeigh

    def reconstructPath(self):
        """ Given the start vertex and goal vertex, and the table of
        predecessors found during the search, this will reconstruct the path
        from start to goal"""
        path = [self.graph.getData(self.goalVert)]
        currVert = self.goalVert
        while currVert != self.startVert:
            currVert = self._pickMinNeighbor(currVert)
            path.insert(0, self.graph.getData(currVert))
        return path

    def reconstructPathNodes(self):
        path = [self.goalVert]
        currVert = self.goalVert
        while currVert != self.startVert:
            currVert = self._pickMinNeighbor(currVert)
            path.insert(0, currVert)
        return path

    def runAlgorithm(self):
        """ Runs algorithm to return the shortest path"""
        route1 = self.computeShortestPath()
        finalPath = self.reconstructPath()
        pathArray = []
        for i in range(len(finalPath) - 1):
            change = [(finalPath[i+1][0] - finalPath[i][0]) * 30, (finalPath[i+1][1] - finalPath[i][1]) * 30]
            if change[0] < 0:
                pathArray.append(['b', -change[0]])
            if change[0] > 0:
                pathArray.append(['f', change[0]])
            if change[0] == 0:
                pathArray.append(['f', 0])
            if change[1] < 0:
                pathArray.append(['r', -change[1]])
            if change[1] > 0:
                pathArray.append(['l', change[1]])
            if change[1] == 0:
                pathArray.append(['f', 0])

        return pathArray, self.reconstructPathNodes()

if __name__ == "__main__":
    mapper = Mapper("macGraph.txt", 23, 40)
    print(mapper.runAlgorithm())
