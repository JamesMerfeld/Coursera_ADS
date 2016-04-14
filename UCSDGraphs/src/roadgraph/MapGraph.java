/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	
	private Map<GeographicPoint, MapNode> nodes;
	
	private int numVertices;
	
	private int numEdges;
	
	private Set<GeographicPoint> nodeSet;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		nodes = new HashMap<GeographicPoint, MapNode>();
		
		numVertices = 0;
		
		numEdges = 0;
		
		nodeSet = new HashSet<GeographicPoint>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{	
		return new HashSet<GeographicPoint>(nodeSet);
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (location == null) {
			
			//Null location reference
			return false;
		}
		
		
		if (nodes.containsKey(location)) {
			
			// Node already in graph
			return false;
		}
		
		nodeSet.add(location);
		
		MapNode newNode = new MapNode(location);
		
		nodes.put(location, newNode);
		
		numVertices++;
		
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		if ( from == null ||
			 to == null ||
			 roadName == null ||
			 roadType == null ||
			 length < 0 ) 
		{
			throw new IllegalArgumentException("Illegal argument");
		}
		
		if (nodes.containsKey(from) && nodes.containsKey(to)) {
			
//			MapNode startNode = nodes.get(from);
			MapNode startNode = nodes.get(to);
			
			startNode.addEdge(to, from, roadName, roadType, length);
			
			numEdges++;
		}
		else
		{
			// One or both of the points have not been added to the graph
			throw new IllegalArgumentException("One or both endpoints do not exist");
		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		
		Queue<MapNode> queue = new LinkedList<MapNode>();
		
		Set<MapNode> visited = new HashSet<MapNode>();
		
		Map<MapNode, MapNode> parent = new HashMap<MapNode, MapNode>();
		
		MapNode curr;
		
		queue.add(nodes.get(start));
		
		visited.add(nodes.get(start));
		
		while (!queue.isEmpty()) {
			
			curr = queue.remove();			
					
			// Check to see if we found the goal node
			if(goal.equals(curr.getPosition())) {
				
				// Found goal!
				List<GeographicPoint> intersections = new ArrayList<GeographicPoint>();
				
				GeographicPoint p = goal;
				
				while (!p.equals(start)) {
					
					intersections.add(p);
					
					p = parent.get(nodes.get(p)).getPosition();
				}
				
				// Add start
				intersections.add(p);
				
				return intersections;
			}
			
			List<MapEdge> neighbors = curr.getEdges();
							
			for (MapEdge neighbor : neighbors) {
				
				MapNode n = nodes.get(neighbor.getDestination());
				
				if (!visited.contains(n)) {
					
					visited.add(n);
					
					System.out.print("n: ["+ n.getPosition().getX() + ", " + n.getPosition().getY() + "]");
					System.out.println(" curr: ["+ curr.getPosition().getX() + ", " + curr.getPosition().getY() + "]");
					
					parent.put(n, curr);
					
					queue.add(n);
				}
			}
		}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		// Didn't find goal
		return null;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.println("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		
		GeographicPoint start = new GeographicPoint(1.0,1.0);
		
		GeographicPoint goal = new GeographicPoint(8.0,-1.0);
		//GeographicPoint goal = new GeographicPoint(4.0,1.0);
		
		System.out.println("Intersections: " + theMap.bfs(start, goal));
		
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
