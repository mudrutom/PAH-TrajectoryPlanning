package student.agent.astar;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;

/**
 * A class used for finding an optimal
 * paths form one node to another in a
 * graph using the A* search algorithm.
 *
 * @param <T> type of node elements
 * @author Tomas Mudrunka
 */
public class AStar<T> {

	private static final double ROOT_COST = 0.0;

	protected final Fringe<T> openList;
	protected final HashSet<AStarNode<T>> closeList;
	protected final DistanceMeasure<T> distanceMeasure;

	private int limit = Integer.MAX_VALUE;

	/**
	 * Creates an instance of AStarNode class
	 * with given distance measure.
	 */
	public AStar(DistanceMeasure<T> distanceMeasure) {
		if (distanceMeasure == null) {
			throw new NullPointerException("distanceMeasure is null");
		}
		openList = new Fringe<T>();
		closeList = new HashSet<AStarNode<T>>();
		this.distanceMeasure = distanceMeasure;
	}

	/**
	 * Sets the upper bound for the number of
	 * iteration done by the algorithm.
	 */
	public void setIterationLimit(int limit) {
		if (limit > 0) {
			this.limit = limit;
		}
	}

	/**
	 * Finds and returns a shortest path from
	 * given start node to given goal node or
	 * returns <tt>null</tt> if no path exists.
	 */
	public List<T> findPath(Node<T> start, Node<T> goal) {
		final double heuristic = distanceMeasure.getDistance(start.getNode(), goal.getNode());
		final AStarNode<T> root = new AStarNode<T>(start, null, ROOT_COST, heuristic);

		// first expand the root
		expandNode(root, goal);

		// main loop of the A* algorithm
		AStarNode<T> node = null;
		int i = limit;
		while (i-- > 0) {
			node = openList.remove();
			if (node == null || goal.equals(node)) {
				break;
			}
			expandNode(node, goal);
			node = null;
		}

		// cleanup
		openList.clear();
		closeList.clear();

		// return a path if the goal was reached
		return (node == null) ? null : buildPath(node, new LinkedList<T>());
	}

	/**
	 * Expands given node if possible.
	 */
	protected void expandNode(AStarNode<T> root, Node<T> goal) {
		if (closeList.contains(root)) {
			// already visited by better path
			return;
		}
		closeList.add(root);

		// add all children into the Fringe
		for (Node<T> node : root.getChildren()) {
			double path = distanceMeasure.getDistance(root.getNode(), node.getNode());
			double heuristic = distanceMeasure.getDistance(node.getNode(), goal.getNode());
			openList.insert(new AStarNode<T>(node, root, path, heuristic));
		}
	}

	/**
	 * Recursively builds a path from the root/start
	 * node to the given node.
	 */
	protected List<T> buildPath(AStarNode<T> node, List<T> path) {
		if (node == null) {
			return path;
		}
		buildPath(node.getParent(), path);
		path.add(node.getNode());
		return path;
	}

	/**
	 * An Interface used by the A* algorithm
	 * to measure the distance between nodes.
	 *
	 * @param <E> type of measured elements
	 */
	public static interface DistanceMeasure<E> {
		/**
		 * Returns the distance between given nodes.
		 */
		public double getDistance(E one, E two);
	}

}
