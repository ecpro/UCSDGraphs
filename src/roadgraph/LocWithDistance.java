package roadgraph;

import java.util.PriorityQueue;

import geography.GeographicPoint;

public class LocWithDistance implements Comparable<LocWithDistance> {

	private GeographicPoint location;
	private Double distance;

	public LocWithDistance(GeographicPoint location, Double distance) {
		this.location = location;
		this.distance = distance;
	}
	
	public boolean isSameLocation(LocWithDistance distance) {
		return location.distance(distance.getLocation()) == 0;
	}

	@Override
	public int compareTo(LocWithDistance o) {
		return this.distance.compareTo(o.distance);
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public Double getDistance() {
		return distance;
	}

	public void setLocation(GeographicPoint location) {
		this.location = location;
	}

	public void setDistance(Double distance) {
		this.distance = distance;
	}

	@Override
	public String toString() {
		return "{" + " (" + location.getX() + "," + location.getY() + ") - " + distance + " }";
	}

	public static void main(String[] args) {

		// for purpose of testing & debugging
		PriorityQueue<LocWithDistance> pq = new PriorityQueue<>(new DistanceComparator(new LocWithDistance(new GeographicPoint(4, 7), 21.0)));
		//PriorityQueue<LocWithDistance> pq = new PriorityQueue<>();
		pq.add(new LocWithDistance(new GeographicPoint(1, 2), 4.0));
		pq.add(new LocWithDistance(new GeographicPoint(2, 3), 1.0));
		pq.add(new LocWithDistance(new GeographicPoint(4, 7), 21.0));
		pq.add(new LocWithDistance(new GeographicPoint(23, 1), 3.0));
		pq.add(new LocWithDistance(new GeographicPoint(21, 56), 0.0));

		System.out.println(pq);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((distance == null) ? 0 : distance.hashCode());
		result = prime * result + ((location == null) ? 0 : location.hashCode());
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
		LocWithDistance other = (LocWithDistance) obj;
		if (distance == null) {
			if (other.distance != null)
				return false;
		} else if (!distance.equals(other.distance))
			return false;
		if (location == null) {
			if (other.location != null)
				return false;
		} else if (!location.equals(other.location))
			return false;
		return true;
	}

}
