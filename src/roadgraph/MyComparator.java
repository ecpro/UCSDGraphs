package roadgraph;

import java.util.Comparator;
import java.util.PriorityQueue;

public class MyComparator implements Comparator<Integer> {

	private Integer goal;
	
	public  MyComparator(Integer goal) {
		this.goal = goal;
	}
	
	@Override
	public int compare(Integer o1, Integer o2) {
		Integer x = o1 + goal;
		Integer y =  o2 + goal;
		
		return x.compareTo(y);
		
	}
	
	public static void main(String[] args) {
		
		PriorityQueue<Integer> pq = new PriorityQueue<>(new MyComparator(10));
		
		pq.add(10);
		pq.add(20);
		pq.add(5);
		pq.add(0);
		pq.add(80);
		
		System.out.println(pq);
		
	}

}
