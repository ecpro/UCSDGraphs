package roadgraph;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

import geography.GeographicPoint;

public class RoadEdgeTEst {
	
	private RoadVertex testVertex;

	@Before
	public void setUp() throws Exception {
		testVertex = new RoadVertex(1.2, 2.2);
	}

	@Test
	public void testAddNeighbour() {
		assertTrue(testVertex.addNeighbor(new GeographicPoint(2, 1)));
		assertFalse(testVertex.addNeighbor(new GeographicPoint(2,1)));
		assertFalse(testVertex.addNeighbor(new GeographicPoint(1.2, 2.2)));
		
		System.out.println(testVertex.getNeighbor());
	}

}
