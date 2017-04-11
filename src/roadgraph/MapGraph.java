/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and Piyush Ravi
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {

	// Intersection of a Set of Roads
	private Map<RoadVertex, Set<RoadEdge>> graphAdjList;

	// for fast RoadVertex Retrieval
	private Map<GeographicPoint, RoadVertex> roadVertexLookup;

	// to store distance at each location
	private Map<GeographicPoint, LocWithDistance> locationDistances;

	private int numVertices;
	private int numEdges;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		graphAdjList = new HashMap<>();
		roadVertexLookup = new HashMap<>();
		numEdges = 0;
		numVertices = 0;
		locationDistances = new HashMap<>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return numVertices;
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return roadVertexLookup.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return numEdges;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {

		if (location != null && !hasLocation(location)) {
			RoadVertex v = new RoadVertex(location);
			graphAdjList.put(v, new HashSet<RoadEdge>());
			roadVertexLookup.put(location, v);
			numVertices++;
			// System.out.println("one vertex added " + v + " numVertices " +
			// numVertices);
			return true;
		}

		System.out.println("cannot add vertex" + location.toString());
		return false;

	}

	/**
	 * check if a location is present as a road vertex
	 * 
	 * @param point
	 * @return boolean
	 */
	public boolean hasLocation(GeographicPoint point) {

		return roadVertexLookup.containsKey(point);
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {
		// check for IllegalArgumentException
		if (from != null && to != null && roadName != null && roadType != null && length >= 0) {
			RoadEdge edge = new RoadEdge(from, to, roadName, roadType, length);
			// find road vertex at "from" location
			RoadVertex rv = roadVertexLookup.get(from);
			// check if road vertex present at the given location and if present
			// then check if edge is also present or not
			if (rv != null && !graphAdjList.get(rv).contains(edge)) {
				graphAdjList.get(rv).add(edge);
				// also add it road vertex neighbor
				rv.addNeighbor(to);
				numEdges++;
				// System.out.println("edge added to between location " +
				// rv.getLocation() + " and " + rv.getLocation());
			} else {
				// System.out.println("road vertex not present or edge already
				// added to vertex");
			}
		} else {
			throw new IllegalArgumentException("invalid argument values");
		}

	}

	public void printAllRoadEdges() {

		for (Map.Entry<RoadVertex, Set<RoadEdge>> entrySet : graphAdjList.entrySet()) {
			for (RoadEdge edge : entrySet.getValue()) {
				System.out.println(edge.getFrom() + " is connected to " + edge.getTo());
			}
		}
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		Map<GeographicPoint, GeographicPoint> traceRouteMap = new HashMap<>();

		boolean found = bfsSearch(start, goal, traceRouteMap, nodeSearched);

		if (!found) {
			System.out.println("path not found");
			return new ArrayList<GeographicPoint>();
		}

		List<GeographicPoint> path = constructPath(traceRouteMap, start, goal);

		Collections.reverse(path);

		return path;

	}

	private List<GeographicPoint> constructPath(Map<GeographicPoint, GeographicPoint> traceRouteMap,
			GeographicPoint start, GeographicPoint goal) {

		List<GeographicPoint> path = new ArrayList<>();

		if (start.distance(goal) == 0) {
			path.add(start);
			return path;
		}

		GeographicPoint current = goal;
		path.add(current);
		while (current.distance(start) != 0) {
			GeographicPoint next = traceRouteMap.get(current);
			current = next;
			path.add(next);
		}

		return path;

	}

	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal,
			Map<GeographicPoint, GeographicPoint> traceRouteMap, Consumer<GeographicPoint> nodeSearched) {

		Queue<GeographicPoint> toExplore = new LinkedList<GeographicPoint>();
		Set<GeographicPoint> visited = new HashSet<>();
		boolean found = false;

		// perform BFS search
		System.out.println("bfs search start ");
		toExplore.add(start);
		// System.out.println(start + " added to the queue");
		visited.add(start);
		while (!toExplore.isEmpty()) {
			GeographicPoint current = toExplore.remove();
			// System.out.println(current + "removed from the queue");
			if (current.distance(goal) == 0) {
				// System.out.println("goal location found" + current + " = " +
				// goal);
				found = true;
				break;
			}

			// Hook for visualization.
			nodeSearched.accept(current);
			List<GeographicPoint> neighbours = roadVertexLookup.get(current).getNeighbor();
			// System.out.println("list of neighbors of " + current + " " +
			// neighbours);
			for (GeographicPoint currGeoPoint : neighbours) {
				if (!visited.contains(currGeoPoint)) {
					toExplore.add(currGeoPoint);
					// System.out.println(currGeoPoint + "added to visitedSet
					// and toExploreSet");
					visited.add(currGeoPoint);
					traceRouteMap.put(currGeoPoint, current);
				}
			}
			// System.out.println("traceRouteMap" + traceRouteMap);
		}
		return found;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		// initialize parentMap && path List
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		boolean found = dijkstraSearch(start, goal, nodeSearched, parentMap);

		System.out.println("Printing parent map in dijkstra " + "length " + parentMap.size());
		for (Entry<GeographicPoint, GeographicPoint> entry : parentMap.entrySet()) {
			System.out.println(
					entry.getKey().x + "," + entry.getKey().y + " -- " + entry.getValue().x + "," + entry.getValue().y);
		}

		List<GeographicPoint> path = null;

		// construct path from start to goal
		if (found) {
			path = constructPath(parentMap, start, goal);
		}

		Collections.reverse(path);

		return path;
	}

	private boolean dijkstraSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched,
			Map<GeographicPoint, GeographicPoint> parentMap) {

		// Initialize distances at each vertex equal to infinity (O(V))
		for (GeographicPoint location : roadVertexLookup.keySet()) {
			locationDistances.put(location, new LocWithDistance(location, Double.POSITIVE_INFINITY));
		}

		// set distance of start to zero
		LocWithDistance startLocatiion = locationDistances.get(start);
		startLocatiion.setDistance(0.0);

		// Initialize Priority Queue and insert start Vertex

		PriorityQueue<LocWithDistance> pq = new PriorityQueue<>();
		pq.add(startLocatiion);

		// Initialize visited set
		Set<GeographicPoint> visited = new HashSet<>();

		while (!pq.isEmpty()) {
			LocWithDistance curr = pq.remove();

			if (!visited.contains(curr)) {
				visited.add(curr.getLocation());
				nodeSearched.accept(curr.getLocation());
				if (curr.getLocation().distance(goal) == 0) {
					System.out.println(visited.size() + " nodes visited in dijkstra " + visited);
					return true;
				}

				for (GeographicPoint neighbor : roadVertexLookup.get(curr.getLocation()).getNeighbor()) {
					if (!visited.contains(neighbor)) {
						LocWithDistance neighbourToCurr = locationDistances.get(neighbor);
						if (curr.getDistance() + curr.getLocation().distance(neighbor) < locationDistances.get(neighbor)
								.getDistance()) {
							neighbourToCurr.setDistance(curr.getLocation().distance(neighbor) + curr.getDistance());
							pq.add(locationDistances.get(neighbor));
							parentMap.put(locationDistances.get(neighbor).getLocation(), curr.getLocation());
						}

					}
				}
			}

			// System.out.print("(" + curr.getLocation().x + "," +
			// curr.getLocation().getY() + ")");

		}
		System.out.println(visited.size() + " nodes visited in dijkstra " + visited);
		return false;

	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();

		boolean found = aStarSearch(start, goal, nodeSearched, parentMap);

		System.out.println("Printing parent map in aStarSearch() " + "length " + parentMap.size());
		for (Entry<GeographicPoint, GeographicPoint> entry : parentMap.entrySet()) {
			System.out.println(
					entry.getKey().x + "," + entry.getKey().y + " -- " + entry.getValue().x + "," + entry.getValue().y);
		}

		List<GeographicPoint> path = new ArrayList<>();

		if (found) {
			path = constructPath(parentMap, start, goal);
		}

		Collections.reverse(path);

		return path;
	}

	private boolean aStarSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched,
			Map<GeographicPoint, GeographicPoint> parentMap) {

		// Initialize distances at each vertex equal to infinity (O(V))
		int count = 0;
		for (GeographicPoint location : roadVertexLookup.keySet()) {
			locationDistances.put(location, new LocWithDistance(location, Double.POSITIVE_INFINITY));
		}

		locationDistances.get(start).setDistance(0.0);

		// Initialize Priority Queue and insert start Vertex

		PriorityQueue<LocWithDistance> pq = new PriorityQueue<>(
				new DistanceComparator(new LocWithDistance(goal, Double.POSITIVE_INFINITY)));
		pq.add(locationDistances.get(start));

		// Initialize visited set
		Set<GeographicPoint> visited = new HashSet<>();

		while (!pq.isEmpty()) {
			LocWithDistance curr = pq.remove();

			if (!visited.contains(curr)) {
				visited.add(curr.getLocation());
				nodeSearched.accept(curr.getLocation());
				if (curr.getLocation().distance(goal) == 0) {
					System.out.println(visited.size() + " nodes visited in astar " + visited);
					return true;
				}

				for (GeographicPoint neighbor : roadVertexLookup.get(curr.getLocation()).getNeighbor()) {
					if (!visited.contains(neighbor)) {
						LocWithDistance neighbourToCurr = locationDistances.get(neighbor);
						if (curr.getDistance() + curr.getLocation().distance(neighbor) < locationDistances.get(neighbor)
								.getDistance()) {
							neighbourToCurr.setDistance(curr.getLocation().distance(neighbor) + curr.getDistance());
							pq.add(locationDistances.get(neighbor));
							parentMap.put(locationDistances.get(neighbor).getLocation(), curr.getLocation());
						}

					}
				}
			}
		}
		System.out.println("total visited nodes " + count);
		return false;

	}

	public void printLocWithDistance() {

		System.out.println("\n");
		for (LocWithDistance loc : locationDistances.values()) {
			System.out.print(loc);
		}
		System.out.println("\n");
	}

	public static void main(String[] args) {
		// System.out.print("Making a new map...");
		// MapGraph firstMap = new MapGraph();
		// System.out.print("DONE. \nLoading the map...");
		// GraphLoader.loadRoadMap("data/maps/utc.map", firstMap);
		// System.out.println("DONE.");
		// System.out.println(firstMap.getVertices());
		// You can use this method for testing.

		// GeographicPoint testStart = new GeographicPoint(32.869423,
		// -117.220917);
		// GeographicPoint testEnd = new GeographicPoint(32.869255,
		// -117.216927);

		// System.out.println("Number of vertices " + firstMap.getNumVertices()
		// + "\nNumber of Edges " + firstMap.getNumEdges());
		// firstMap.printAllRoadEdges();

		// List<GeographicPoint> path = firstMap.bfs(testStart, testEnd);

		// System.out.println(path);

		/*
		 * Here are some test cases you should try before you attempt the Week 3
		 * End of Week Quiz, EVEN IF you score 100% on the programming
		 * assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5\n");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart, testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart, testEnd);

		// System.out.println("First Test Results : " + "dijkstra " +
		// testroute.size() + " aStar " + testroute2.size());

		System.out.println("dijkstra " + testroute);

		// simpleTestMap.printLocWithDistance();

		System.out.println("astar    " + testroute2);

		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart, testEnd);
		System.out.println("second Test Results : " + "dijkstra " + testroute.size() + " aStar " + testroute2.size());

		System.out.println(testroute);
		System.out.println(testroute2);

		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart, testEnd);

		System.out.println("3rd Test Results : " + "dijkstra " + testroute.size() + " aStar " + testroute2.size());

		System.out.println(testroute);
		System.out.println(testroute2);

		System.out.println("questions -------------------");

		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start, end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start, end);
		
		System.out.println(testroute);
		System.out.println(testroute2);
		
		System.out.println("questions -------------------");


	}

}
