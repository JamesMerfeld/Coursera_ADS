package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	
	private GeographicPoint start;

	private GeographicPoint end;
	
	private String roadName;
	
	private String roadType;
	
	private Double distance;
	
	public MapEdge(GeographicPoint from, GeographicPoint to, String name,
			String type, double length) {
		
		start = new GeographicPoint(from.getX(), from.getY());
		
		end = new GeographicPoint(to.getX(), to.getY());
		
		roadName = name;
		
		roadType = type;
		
		distance = length;
	}
}
