package student.agent;

import com.jme3.math.Vector3f;
import cz.agents.alite.pahtactical.agent.PAHCarAgent;
import cz.agents.alite.pahtactical.vis.PlanLayer;
import cz.agents.alite.tactical.universe.environment.TacticalEnvironment.TacticalEnvironmentHandler;
import cz.agents.alite.tactical.universe.world.map.Building;
import cz.agents.alite.tactical.universe.world.map.UrbanMap;
import cz.agents.alite.tactical.universe.world.physics.Terrain;
import cz.agents.alite.tactical.util.Point;
import cz.agents.alite.tactical.util.Polygon2d;
import cz.agents.alite.tactical.util.Visibility;
import cz.agents.alite.vis.VisManager;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import student.agent.astar.AStar;
import student.agent.astar.Node;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import java.awt.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;

/**
 * Implementation of PAHCarAgent class used to control an agent (UGV)
 * in the test simulation. Its goal is to visit all provided checkpoints
 * and to minimize the distance traveled.
 *
 * <p>Implementation notes:<br/>
 * <ul>
 *     <li>it uses the Rapidly-exploring Random Graph (RRG) algorithm to
 *         generate a node-graph used for finding a path</li>
 *     <li>a path between checkpoints is found by the A* search algorithm</li>
 *     <li>checkpoints are greedy-sorted to minimize a distance traveled</li>
 *     <li>new Visibility class with inflated buildings is used</li>
 *     <li>if the UGV gets stuck by a building it will turn sideways</li>
 *     <li>if the node-graph is disconnected, so a path couldn't be found,
 *         new near-checkpoint nodes are generated and added to the graph</li>
 * </ul>
 * </p>
 *
 * @author Tomas Mudrunka
 */
public class StudentPAHCarAgent extends PAHCarAgent {

	private static final Logger LOGGER = Logger.getLogger(StudentPAHCarAgent.class);
	private static final boolean VISUALIZATION = true;
	private static final boolean DEBUG = true;

	protected static final float CAR_SPEED = 30.0f;
	protected static final double Z_OFFSET = 1.0;
	protected static final double INFLATE_DELTA = 1.2;
	protected static final int NODE_NUM = 6000;
	protected static final double X_max = 1170.0, X_min = 180.0, Y_max = 1000.0, Y_min = 180.0;
	protected static final double RRG_GAMA = 2500.0, RRG_MIN_RADIUS = 50.0;
	protected static final int TICK_DAMPING = 5;
	protected static final float TURN_POINT_DIST = 7.0f;
	protected static final int EXTRA_NODE_NUM = 20;
	protected static final double EXTRA_NODE_SIGMA = 25.0, EXTRA_NODE_RADIUS = 250.0;

	private List<Polygon2d> buildings;
	private Visibility visibility;

	private Collection<Node<Point>> nodes;
	private List<Node<Point>> checkPoints;
	private List<Point> pointsToGo;

	private AStar<Point> aStar;

	private boolean finished, moving;
	private Point prevPosition;
	private int tickCount;
	private long t0;

	/**
	 * Constructor of the StudentPAHCarAgent class.
	 */
	public StudentPAHCarAgent(String name, UrbanMap map, Terrain terrain, TacticalEnvironmentHandler handler, Set<Point> checkpoints) {
		super(name, map, terrain, handler, checkpoints);
		if (DEBUG) {
			LOGGER.setLevel(Level.DEBUG);
		} else {
			LOGGER.setLevel(Level.INFO);
		}
	}

	@Override
	protected void ready() {
		LOGGER.info("Initialization started...");
		finished = false;
		moving = false;
		prevPosition = null;
		tickCount = 0;
		t0 = System.currentTimeMillis();

		initBuildings();
		nodes = initNodesRRG();

		LOGGER.debug("Checkpoint pre-processing started");
		checkPoints = new ArrayList<Node<Point>>(getCheckpoints().size());
		for (Point checkpoint : getCheckpoints()) {
			Node<Point> checkpointNode = new Node<Point>(checkpoint);
			for (Node<Point> n : nodes) {
				if (visibility.isVisible(checkpoint, n.getNode(), Double.MAX_VALUE)) {
					n.addChild(checkpointNode);
					checkpointNode.addChild(n);
				}
			}
			checkPoints.add(checkpointNode);
		}
		nodes.addAll(checkPoints);
		sortCheckpoints();

		aStar = new AStar<Point>(new AStar.DistanceMeasure<Point>() {
			@Override
			public double getDistance(Point one, Point two) {
				return one.distance(two);
			}
		});
		aStar.setIterationLimit(nodes.size());

		registerVisualization();

		LOGGER.info("Initialization finished in " + (System.currentTimeMillis() - t0) + "ms");
		goToNextCheckPoint();
	}

	@Override
	protected void waypointReached(Point waypoint) {
		if (waypoint != null && waypoint.epsilonEquals(pointsToGo.get(0), 1.0)) {
			LOGGER.debug("Waypoint " + waypoint + " has been reached");
			moving = false;
			halt();
			pointsToGo.remove(0);
			if (pointsToGo.isEmpty()) {
				LOGGER.info("Next checkpoint has been reached");
				goToNextCheckPoint();
			} else {
				goToWaypoint(pointsToGo.get(0), CAR_SPEED, true);
				moving = true;
			}
		}
	}

	@Override
	protected void positionChanged(Point newPosition) {
		if (!finished && moving && newPosition != null) {
			// save the current position
			prevPosition = newPosition;
		}
	}

	@Override
	protected void tick(long simulationTime) {
		// test for getting stuck after every few ticks
		if (tickCount++ > TICK_DAMPING && !finished && moving && prevPosition != null) {
			final Point position = getCurrentPosition();
			if (prevPosition.epsilonEquals(position, 0.01)) {
				moving = false;
				halt();
				resolveGettingStuck();
			}
			tickCount = 0;
		}
	}

	/**
	 * Finds a path and navigates to the next checkpoint if any.
	 */
	private void goToNextCheckPoint() {
		if (checkPoints.isEmpty()) {
			LOGGER.info("All given checkpoints have been reached");
			LOGGER.debug("Total time spent: " + (System.currentTimeMillis() - t0) + "ms");
			finished = true;
			return;
		}

		// add the current position into the node-graph
		final Node<Point> position = new Node<Point>(getCurrentPosition());
		for (Node<Point> n : nodes) {
			if (visibility.isVisible(position.getNode(), n.getNode(), Double.MAX_VALUE)) {
				n.addChild(position);
				position.addChild(n);
			}
		}
		final Node<Point> nextCheckPoint = checkPoints.remove(0);

		LOGGER.debug("Initiating A* search from " + position.getNode() + " to " + nextCheckPoint.getNode());
		final long t = System.currentTimeMillis();
		pointsToGo = aStar.findPath(position, nextCheckPoint);
		if (pointsToGo == null) {
			resolveNoPathFound(position, nextCheckPoint);
			return;
		}
		LOGGER.debug("Path of length " + pointsToGo.size() + " found in " + (System.currentTimeMillis() - t) + "ms");

		goToWaypoint(pointsToGo.get(0), CAR_SPEED, true);
		moving = true;
	}

	/**
	 * Greedy sorting of the checkpoints to minimize a distance traveled.
	 */
	private void sortCheckpoints() {
		Point current = getCurrentPosition();
		for (int i = 0; i < checkPoints.size(); i++) {
			double bestDist = current.distance(checkPoints.get(i).getNode());
			for (int j = i + 1; j < checkPoints.size(); j++) {
				double dist = current.distance(checkPoints.get(j).getNode());
				if (bestDist > dist) {
					// swap if a better path is found
					Node<Point> worse = checkPoints.get(i);
					checkPoints.set(i, checkPoints.get(j));
					checkPoints.set(j, worse);
					bestDist = dist;
				}
			}
			current = checkPoints.get(i).getNode();
		}
	}

	/**
	 * Will inflate the buildings and constructs a new Visibility class.
	 */
	private void initBuildings() {
		buildings = new ArrayList<Polygon2d>(getMap().getBuildings().size());
		for (Building b : getMap().getBuildings()) {
			buildings.add(new Polygon2d(b.getBaseAsPolygon2d().inflate(INFLATE_DELTA)));
		}
		visibility = new Visibility(buildings);
	}

	/**
	 * Initialization of the node-graph, returns the collection of nodes.
	 * The Rapidly-exploring Random Graph (RRG) algorithm is used to
	 * generate the node-graph of desired size.
	 */
	private Collection<Node<Point>> initNodesRRG() {
		final Random randomX = new Random(System.currentTimeMillis());
		final Random randomY = new Random(System.currentTimeMillis() >>> 32);

		int edgeCount = 0;

		final List<Node<Point>> graph = new ArrayList<Node<Point>>(NODE_NUM);
		while (graph.size() < NODE_NUM) {
			// generate a point uniformly at random
			double x = randomX.nextDouble() * X_max + X_min;
			double y = randomY.nextDouble() * Y_max + Y_min;
			Point rndPoint = new Point(x, y, getAltitude(x, y) + Z_OFFSET);
			// test whether it's valid
			if (!Double.isNaN(rndPoint.z) && !visibility.isInBuilding(rndPoint)) {
				Node<Point> rndNode = new Node<Point>(rndPoint);
				// check visibility with all other points within the radius
				double radius = RRG_GAMA * Math.sqrt(Math.log(graph.size()) / graph.size());
				radius = (Double.isNaN(radius) || radius < RRG_MIN_RADIUS) ? RRG_MIN_RADIUS : radius;
				if (graph.size() % (NODE_NUM / 10) == 0) {
					LOGGER.debug("RRG algorithm -- #nodes=" + graph.size() + " radius=" + radius);
				}
				for (Node<Point> node : graph) {
					if (visibility.isVisible(rndPoint, node.getNode(), radius)) {
						// connect nodes to each other
						node.addChild(rndNode);
						rndNode.addChild(node);
						edgeCount++;
					}
				}
				graph.add(rndNode);
			}
		}

		LOGGER.debug("Node-graph constructed with " + graph.size() + " nodes and " + edgeCount + " edges");
		LOGGER.debug("Node-graph construction time: " + (System.currentTimeMillis() - t0) + "ms");

		return graph;
	}

	/**
	 * Resolves the problem where the car has got stuck at one position.
	 * This mostly happens when it collides with an obstacle. In such case
	 * it will turn (change its heading) to the left, right or back.
	 */
	private void resolveGettingStuck() {
		LOGGER.debug("The car has got stuck.");
		final Point position = getCurrentPosition();
		final Vector3f heading = getCurrentHeading().normalize();

		// compute possible turning points
		final Vector3f vSide = heading.crossLocal(0.0f, 0.0f, 1.0f).mult(TURN_POINT_DIST);
		final Vector3f vBack = heading.mult(TURN_POINT_DIST / 2.0f);
		final Point side1 = new Point(position);
		final Point side2 = new Point(position);
		final Point back = new Point(position);
		side1.add(new Point3d(vSide.x, vSide.y, getAltitude(vSide.x, vSide.y) + Z_OFFSET));
		side2.sub(new Point3d(vSide.x, vSide.y, getAltitude(vSide.x, vSide.y) + Z_OFFSET));
		back.sub(new Point3d(vBack.x, vBack.y, getAltitude(vBack.x, vBack.y) + Z_OFFSET));

		double dist1 = Double.MAX_VALUE, dist2 = Double.MAX_VALUE;
		if (!visibility.isInBuilding(side1) && visibility.isVisible(position, side1)) {
			dist1 = side1.distance(pointsToGo.get(0));
		}
		if (!visibility.isInBuilding(side2) && visibility.isVisible(position, side2)) {
			dist2 = side2.distance(pointsToGo.get(0));
		}

		// decide where to turn
		if (dist1 <= dist2 && dist1 != Double.MAX_VALUE) {
			LOGGER.debug("Turning to a side point " + side1);
			pointsToGo.add(0, side1);
		} else if (dist2 != Double.MAX_VALUE) {
			LOGGER.debug("Turning to a side point " + side2);
			pointsToGo.add(0, side2);
		} else {
			// it's assumed that this point is always valid
			LOGGER.debug("Turning back to point " + back);
			pointsToGo.add(0, back);
		}

		goToWaypoint(pointsToGo.get(0), CAR_SPEED, true);
		moving = true;
	}

	/**
	 * Resolve the problem where there is no path found from the current
	 * position to the next checkpoint. This happens when the node-graph
	 * is disconnected. In such case new near-checkpoint nodes are generated
	 * and added into the graph until it becomes connected (a path is found).
	 */
	private void resolveNoPathFound(Node<Point> position, Node<Point> checkpoint) {
		LOGGER.debug("No path has been found, the node-graph is disconnected!");
		final Random randomX = new Random(System.currentTimeMillis());
		final Random randomY = new Random(System.currentTimeMillis() >>> 32);

		int edgeCount = 0;

		// generate extra nodes near the checkpoint
		final List<Node<Point>> extraNodes = new ArrayList<Node<Point>>(EXTRA_NODE_NUM);
		while (extraNodes.size() < EXTRA_NODE_NUM) {
			double x = randomX.nextGaussian() * EXTRA_NODE_SIGMA + checkpoint.getNode().x;
			double y = randomY.nextGaussian() * EXTRA_NODE_SIGMA + checkpoint.getNode().y;
			Point rndPoint = new Point(x, y, getAltitude(x, y) + Z_OFFSET);
			if (!Double.isNaN(rndPoint.z) && !visibility.isInBuilding(rndPoint)) {
				Node<Point> rndNode = new Node<Point>(rndPoint);
				if (visibility.isVisible(rndPoint, checkpoint.getNode())) {
					checkpoint.addChild(rndNode);
					rndNode.addChild(checkpoint);
					edgeCount++;
				}
				for (Node<Point> node : nodes) {
					if (visibility.isVisible(rndPoint, node.getNode(), EXTRA_NODE_RADIUS)) {
						node.addChild(rndNode);
						rndNode.addChild(node);
						edgeCount++;
					}
				}
				for (Node<Point> node : extraNodes) {
					if (visibility.isVisible(rndPoint, node.getNode(), EXTRA_NODE_RADIUS)) {
						node.addChild(rndNode);
						rndNode.addChild(node);
						edgeCount++;
					}
				}
				extraNodes.add(rndNode);
			}
		}
		LOGGER.debug("New near-checkpoint nodes added to the graph, with " + edgeCount + " edges");

		pointsToGo = aStar.findPath(position, checkpoint);
		if (pointsToGo == null) {
			// recourse until a path is found
			resolveNoPathFound(position, checkpoint);
			return;
		}
		LOGGER.debug("Path of length " + pointsToGo.size() + " found");

		goToWaypoint(pointsToGo.get(0), CAR_SPEED, true);
		moving = true;
	}

	/**
	 * Registers 2D visualization layers.
	 */
	private void registerVisualization() {
		if (VISUALIZATION) {
			LOGGER.debug("Registering BUILDING visualization layers");
			for (Polygon2d b : buildings) {
				final List<Point> plan = new LinkedList<Point>();
				for (Point2d p : b.getPoints()) {
					plan.add(new Point(p.x, p.y, getAltitude(p.x, p.y) + Z_OFFSET));
				}
				plan.add(plan.get(0));
				VisManager.registerLayer(PlanLayer.create(new PlanLayer.PlanProvider() {
					@Override
					public List<Point> getPlan() {
						return plan;
					}
				}, Color.GREEN, 1));
			}

			LOGGER.debug("Registering NODE-GRAPH visualization layers");
			for (Node<Point> node : nodes) {
				final List<Point> plan = new LinkedList<Point>();
				for (Node<Point> p : node.getChildren()) {
					plan.add(node.getNode());
					plan.add(p.getNode());
				}
				VisManager.registerLayer(PlanLayer.create(new PlanLayer.PlanProvider() {
					@Override
					public List<Point> getPlan() {
						return plan;
					}
				}, Color.LIGHT_GRAY, 1));
			}

			LOGGER.debug("Registering CHECKPOINT visualization layer");
			final List<Point> plan = new LinkedList<Point>();
			for (Node<Point> p : checkPoints) {
				plan.add(p.getNode());
			}
			VisManager.registerLayer(PlanLayer.create(new PlanLayer.PlanProvider() {
				@Override
				public List<Point> getPlan() {
					return plan;
				}
			}, Color.BLUE, 2));

			LOGGER.debug("Registering PATH visualization layer");
			VisManager.registerLayer(PlanLayer.create(new PlanLayer.PlanProvider() {
				@Override
				public List<Point> getPlan() {
					final List<Point> plan = new LinkedList<Point>();
					if (getCurrentPosition() != null) {
						plan.add(getCurrentPosition());
					}
					if (pointsToGo != null) {
						plan.addAll(pointsToGo);
					}
					return plan;
				}
			}, Color.RED, 2));
		}
	}

}
