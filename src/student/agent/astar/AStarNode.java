package student.agent.astar;

import java.util.List;

/**
 * Extension of the basic {@link Node} class
 * with features required by the A* algorithm.
 *
 * @param <T> type of stored element
 * @author Tomas Mudrunka
 */
public class AStarNode<T> extends Node<T> implements Comparable<AStarNode<T>> {

	private final Node<T> node;
	private final AStarNode<T> parent;

	private final double g, f;

	/**
	 * Constructs A* node for given basic node
	 * and parent A* node.
	 *
	 * @param pathCost  the cost of path form the parent node
	 * @param heuristic the estimated cost of a path to the goal
	 */
	public AStarNode(Node<T> node, AStarNode<T> parent, double pathCost, double heuristic) {
		super(node.getNode());
		this.node = node;
		this.parent = parent;
		g = (parent == null) ? pathCost : parent.g + pathCost;
		f = g + heuristic;
	}

	/**
	 * Returns the parent node of this node.
	 */
	public AStarNode<T> getParent() {
		return parent;
	}

	/**
	 * Returns cost <tt>f(N)</tt> of this node.
	 * <p><code>f(N) = g(N) + h(N)</code></p>
	 * Where <tt>g(N)</tt> is a cost of the best
	 * path to this node and <tt>h(N)</tt> is an
	 * estimated cost from this node to the goal.
	 */
	public double getCost() {
		return f;
	}

	@Override
	public void addChild(Node<T> child) {
		node.addChild(child);
	}

	@Override
	public List<Node<T>> getChildren() {
		return node.getChildren();
	}

	@Override
	public int compareTo(AStarNode<T> node) {
		// for sorting performance equal case is considered as greater
		return (f > node.f) ? 1 : -1;
	}

}
