Class: MapGraph

Modifications made to MapGraph (what and why): 

private Map<RoadVertex, Set<RoadEdge>> graphAdjList;

private Map<GeographicPoint, RoadVertex> roadVertexLookup;

private int numVertices;

private int numEdges;

Following fields have been added to the class. First field stores graph as adjacency list. For storing intersections a class called RoadVertex 
has been written and for edges RoadEdge has been written which is explained below. 
Since a road can only be unique, SET data structure have been used to store them instead of lists. 
Field roadVertexLookup is just for fast retrieval of roadVertex.

BFS has been refactored into bfsSearch() and constructPath() for modularity and seperation of concern.

Two new methods have been introduced hasLocation() and printAllEdges() as helper function and mostly for debugging.

Class RoadVertex :

It has Geographic point and a set of neighbouring vertices and the class symoolizes a intersection or edge end point.
It has method addNeighbour() to keep track of all the connected neighbouring vertices. 

Class RoadEdge:

This class basically symbolises a map edge/road representing roads between intersections or say vertices.

