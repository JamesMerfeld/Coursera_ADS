package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	
	private GeographicPoint position;
	
	private List<MapEdge> edges;
	
	public MapNode(GeographicPoint p) {
		
		position = new GeographicPoint(p.getX(), p.getY());
		
		edges = new ArrayList<MapEdge>();
	}
	
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) {
		
		MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
		
		edges.add(newEdge);	
	}

}
