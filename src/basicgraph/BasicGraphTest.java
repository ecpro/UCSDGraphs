package basicgraph;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

import junit.framework.Assert;
import util.GraphLoader;

public class BasicGraphTest {
	
	private Graph testGraph;

	@Before
	public void setUp() throws Exception {
		testGraph  = new GraphAdjList();
		GraphLoader.loadRoadMap("data/testdata/myTestdata.map", testGraph);
	}

	@Test
	public void test() {
		int numEdges = testGraph.getNumEdges();
		int numVertices = testGraph.getNumVertices();
		try {
			assertEquals(3, numVertices);
			assertEquals(4, numEdges);
		} catch (Exception e) {
			fail("cannot match the criteria");
		}
	}

}
