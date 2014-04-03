package student.creator;

import cz.agents.alite.pahtactical.creator.DefaultCreator;
import cz.agents.alite.tactical.util.Point;
import org.apache.log4j.Logger;

import java.util.LinkedHashSet;
import java.util.Set;

/**
 *  Class used for creating a new testing simulations.
 *
 *  @author Tomas Mudrunka
 */
public class StudentTestingCreator extends DefaultCreator {

	private static final Logger LOGGER = Logger.getLogger(StudentTestingCreator.class);
	private static final double Z_OFFSET = 1.5;

	/**
	 * Main method, initiates and starts the simulation.
	 */
	public static void main(String[] args) {
		final StudentTestingCreator creator = new StudentTestingCreator();
		creator.init(args);
		creator.enableVis2d(true);
		creator.enableVis3d(false);
		creator.create();
	}

	@Override
	protected void createEntities() {
		LOGGER.info(">>> CREATING UGV ENTITY");

		// setup the checkpoints
		final Point initLocation = newPoint(530, 800);
		final Set<Point> checkpoints = new LinkedHashSet<Point>();
//		checkpoints.add(newPoint(910,706));
//		checkpoints.add(newPoint(931,841));
//		checkpoints.add(newPoint(966,774.5));
//		checkpoints.add(newPoint(974, 800));

		checkpoints.add(newPoint(646, 433));
		checkpoints.add(newPoint(651, 520));
		checkpoints.add(newPoint(780, 575));
		checkpoints.add(newPoint(835, 710));
		checkpoints.add(newPoint(1056, 427));
		checkpoints.add(newPoint(787, 504));
		checkpoints.add(newPoint(858.5, 345.7));
		checkpoints.add(newPoint(940, 854));
		checkpoints.add(newPoint(867, 796));
		checkpoints.add(newPoint(1225, 368));

		// add the UGV
		addCarAgent(initLocation, checkpoints);
	}

	/**
	 * Returns a new Point with given XY coordinates.
	 */
	public Point newPoint(double x, double y) {
		return new Point(x, y, getAltitude(x, y) + Z_OFFSET);
	}

}
