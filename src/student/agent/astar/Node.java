package student.agent.astar;

import java.util.LinkedList;
import java.util.List;

/**
 * A class representing one node in the graph-search-space.
 *
 * @param <T> type of stored element
 * @author Tomas Mudrunka
 */
public class Node<T> {

	private final T node;
	private final List<Node<T>> children;

	/**
	 * Constructs node for given element.
	 */
	public Node(T node) {
		if (node == null) {
			throw new NullPointerException("node is null");
		}
		this.node = node;
		children = new LinkedList<Node<T>>();
	}

	/**
	 * Returns an element of this node.
	 */
	public T getNode() {
		return node;
	}

	/**
	 * Adds given child node to this node.
	 */
	public void addChild(Node<T> child) {
		if (child != null) {
			children.add(child);
		}
	}

	/**
	 * Returns a list of all child nodes of this node.
	 */
	public List<Node<T>> getChildren() {
		return children;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (!(o instanceof Node)) return false;

		return node.equals(((Node) o).node);
	}

	@Override
	public int hashCode() {
		return node.hashCode();
	}

}
