package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class GeoLocation {
	
	private GeographicPoint coordinate;
	private String locationAreaName;
	private List<Road> roads;

	public GeoLocation(GeographicPoint coordinate) {
		this.coordinate = coordinate;
		this.locationAreaName = "unnamed";
		this.setRoads(new ArrayList<Road>());
	}

	public GeoLocation(double latitude, double longitude) {
		this.coordinate = new GeographicPoint(latitude, longitude);
		this.locationAreaName = "unnamed";
		this.setRoads(new ArrayList<Road>());
	}

	public void setLocationByCoordinates(double latitude, double longitude) {
		this.coordinate = new GeographicPoint(latitude, longitude);
	}

	public GeographicPoint getCoordinate() {
		return coordinate;
	}

	public void setCoordinate(GeographicPoint coordinate) {
		this.coordinate = coordinate;
	}

	public String getLocationName() {
		return locationAreaName;
	}

	public void setLocationNames(String locationName) {
		this.locationAreaName = locationName;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((coordinate == null) ? 0 : coordinate.hashCode());
		result = prime * result + ((locationAreaName == null) ? 0 : locationAreaName.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		GeoLocation other = (GeoLocation) obj;
		if (coordinate == null) {
			if (other.coordinate != null)
				return false;
		} else if (!coordinate.equals(other.coordinate))
			return false;
		if (locationAreaName == null) {
			if (other.locationAreaName != null)
				return false;
		} else if (!locationAreaName.equals(other.locationAreaName))
			return false;
		return true;
	}

	public List<Road> getRoads() {
		return roads;
	}

	public void setRoads(List<Road> roads) {
		this.roads = roads;
	}

}
