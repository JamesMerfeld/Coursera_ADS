package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode>{
	
	private GeographicPoint position;
	
	private List<MapEdge> edges;
	
	private double distanceFromStart; 
	
	public MapNode(GeographicPoint p) {
		
		position = new GeographicPoint(p.getX(), p.getY());
		
		edges = new ArrayList<MapEdge>();
		
		distanceFromStart = 0;
	}

	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) {
		
		MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
		
		edges.add(newEdge);	
	}
	
	public List<MapEdge> getEdges() {
		
		return new ArrayList<MapEdge>(edges);
	}
	
	public GeographicPoint getPosition() {
		
		return new GeographicPoint(position.getX(), position.getY()); 
	}
	
	public double getDistanceFromStart() {
		
		return distanceFromStart;
	}
	
	public void setDistanceFromStart(double d) {
		
		distanceFromStart = d;
	}

	@Override
	public int compareTo(MapNode node) {
		
		double d = node.getDistanceFromStart(); 
		
	    if (this.distanceFromStart < d) {
	        return -1;
	    }
	    else {
	    	return 1;
	    }
	}

}
