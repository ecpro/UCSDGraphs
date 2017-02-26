package roadgraph;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import geography.GeographicPoint;

/** 
 * 	RoadVertex class represent the road intersection and road endpoint
 * 
 * @author Piyush Ravi
 *
 */

public class RoadVertex{
	
	private GeographicPoint location;
	
	// Using Set as neighbors cannot be duplicate
	private Set<GeographicPoint> neighbors;

	public RoadVertex(GeographicPoint location) {
		this.location = location;
		this.neighbors = new HashSet<GeographicPoint>();
	}

	public RoadVertex(double latitude, double longitude) {
		this.location = new GeographicPoint(latitude, longitude);
		this.neighbors = new HashSet<GeographicPoint>();
	}

	
	public RoadVertex(GeographicPoint location, Set<GeographicPoint> neighbors) {
		this.location = location;
		this.neighbors = neighbors;
	}
	
	public GeographicPoint getLocation() {
		return location;
	}
	
	public void addNeighbor(GeographicPoint neighbor) {
		// same Location also cannot be neighbor
		if(!isSameLocation(neighbor) && !neighbors.contains(neighbor)) {
			neighbors.add(neighbor);
			// for debugging purpose
			System.out.println(neighbor.toString() + " add to the neighborList");
		}
		else {
			// for debugging purpose
			System.out.println(neighbor.toString() + " already present or sameLocation = " + isSameLocation(neighbor));
		}
	}
	
	public List<GeographicPoint> getNeighbor() {
		List<GeographicPoint> neighborList = new ArrayList<>();
			for(GeographicPoint point : neighbors) {
				neighborList.add(point);
			}
			return neighborList;
	}
	
	public boolean isSameLocation(GeographicPoint location) {
		return this.location.distance(location) == 0;
	}

}
