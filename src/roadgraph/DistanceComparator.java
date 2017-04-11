package roadgraph;

import java.util.Comparator;

public class DistanceComparator implements Comparator<LocWithDistance> {
	
	private LocWithDistance goal;
	
	public DistanceComparator(LocWithDistance loc) {
		this.goal = loc;
	}

	@Override
	public int compare(LocWithDistance o1, LocWithDistance o2) {
		
		Double o1_to_goal = o1.getDistance() + goal.getLocation().distance(o1.getLocation());
		Double o2_to_goal = o2.getDistance() + goal.getLocation().distance(o2.getLocation());
		
		return o1_to_goal.compareTo(o2_to_goal);
	}

}
