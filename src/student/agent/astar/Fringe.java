package student.agent.astar;

import java.util.HashSet;
import java.util.PriorityQueue;

/**
 * Fringe used as an open-list in A* searching algorithm.
 *
 * @param <T> type of stored elements
 * @author Tomas Mudrunka
 */
public class Fringe<T> {

	private final PriorityQueue<AStarNode<T>> queue;
	private final HashSet<AStarNode<T>> hashSet;

	public Fringe() {
		queue = new PriorityQueue<AStarNode<T>>();
		hashSet = new HashSet<AStarNode<T>>();
	}

	/**
	 * Inserts given element into to the Fringe. The fringe
	 * will not accept duplicate elements.
	 */
	public void insert(AStarNode<T> node) {
		if (node != null && !hashSet.contains(node)) {
			queue.add(node);
			hashSet.add(node);
		}
	}

	/**
	 * Removes and returns the first element from the Fringe
	 * or returns <tt>null</tt> if it is empty.
	 */
	public AStarNode<T> remove() {
		final AStarNode<T> node = queue.poll();
		hashSet.remove(node);
		return node;
	}

	/**
	 * Removes all the elements from the Fringe.
	 */
	public void clear() {
		queue.clear();
		hashSet.clear();
	}

	/**
	 * Returns the number of elements in the Fringe.
	 */
	public int size() {
		return queue.size();
	}

}
