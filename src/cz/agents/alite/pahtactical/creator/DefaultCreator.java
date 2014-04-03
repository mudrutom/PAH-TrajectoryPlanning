package cz.agents.alite.pahtactical.creator;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.pahtactical.PAHUniverse;
import cz.agents.alite.pahtactical.agent.PAHCarAgent;
import cz.agents.alite.simulation.Simulation;
import cz.agents.alite.tactical.creator.config.DefaultVillageConfig;
import cz.agents.alite.tactical.creator.config.VillageConfig;
import cz.agents.alite.tactical.universe.entity.embodiment.Team;
import cz.agents.alite.tactical.universe.environment.TacticalEnvironment;
import cz.agents.alite.tactical.util.Point;
import cz.agents.alite.tactical.vis.BubbleLayer;
import cz.agents.alite.tactical.vis.BuildingLayer;
import cz.agents.alite.tactical.vis.CarLayer;
import cz.agents.alite.tactical.vis.SimulationControlLayer;
import cz.agents.alite.tactical.vis.ZoneLayer;
import cz.agents.alite.tactical.vis3d.layer.Mdars3dLayer;
import cz.agents.alite.tactical.vis3d.layer.Person3dLayer;
import cz.agents.alite.tactical.vis3d.layer.Procerus3dLayer;
import cz.agents.alite.tactical.vis3d.layer.Procerus3dLayer.VisibilityConeType;
import cz.agents.alite.tactical.vis3d.layer.Skeldar3dLayer;
import cz.agents.alite.tactical.vis3d.layer.TacticalPerson3dLayer;
import cz.agents.alite.tactical.vis3d.layer.Vidar3dLayer;
import cz.agents.alite.tactical.vis3d.layer.Village3dLayer;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.FpsLayer;
import cz.agents.alite.vis.layer.common.HelpLayer;
import cz.agents.alite.vis.layer.common.LogoLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;
import cz.agents.alite.vis3d.Vis3dManager;
import cz.agents.alite.vis3d.layer.SimulationControl3dLayer;
import cz.agents.alite.vis3d.layer.Skybox3dLayer;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.GnuParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.OptionBuilder;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import student.agent.StudentPAHCarAgent;

import java.awt.*;
import java.io.File;
import java.net.URISyntaxException;
import java.util.Random;
import java.util.Set;

public abstract class DefaultCreator implements Creator {


	public static final String OPT_NOVIS = "novis";
	public static final String OPT_NOVIS3D = "novis3d";
	public static final String OPT_CONF = "conf";
	public static final String OPT_HELP = "help";

	public static int carCounter = 0;

	private static final Logger LOGGER = Logger.getLogger(DefaultCreator.class);
	private static final long RANDOM_SEED = 0;
	private PAHUniverse universe;

	private boolean loadVis = true;
	private boolean loadVis3D = true;
	private VillageConfig villageConfig = new DefaultVillageConfig();
	private static final String LOGO_IMAGE_FILE = "img/atg_blue.png";

	@Override
	public void init(String[] args) {
		Logger.getRootLogger().setLevel(Level.INFO);
		parseArgs(args);
	}

	private void parseArgs(String[] args) {
		Options options = loadOptions();

		CommandLineParser parser = new GnuParser();

		try {
			CommandLine line = parser.parse(options, args);

			if (line.hasOption(OPT_HELP)) {
				printHelp(options);
				System.exit(0);
			}

			if (line.hasOption(OPT_NOVIS)) {
				loadVis = false;
			}

			if (line.hasOption(OPT_NOVIS3D)) {
				loadVis3D = false;
			}

			if (line.hasOption(OPT_CONF)) {
				String path = line.getOptionValue(OPT_CONF);
				try {
					villageConfig = (VillageConfig) Class.forName(path).newInstance();
				} catch (InstantiationException ex) {
					LOGGER.error("invalid " + OPT_CONF + " argument", ex);
					printHelp(options);
					System.exit(1);
				} catch (IllegalAccessException ex) {
					LOGGER.error("invalid " + OPT_CONF + " argument", ex);
					printHelp(options);
					System.exit(1);
				} catch (ClassNotFoundException ex) {
					LOGGER.error("invalid " + OPT_CONF + " argument", ex);
					printHelp(options);
					System.exit(1);
				}
			}
		} catch (ParseException e) {
			LOGGER.error("Parsing of input arguments failed.", e);
			printHelp(options);
			System.exit(1);
		}

	}

	private void printHelp(Options options) {
		HelpFormatter helpFormatter = new HelpFormatter();

		String name = "tactical.jar";

		try {
			File jarFile = new File(DefaultCreator.class.getProtectionDomain().getCodeSource().getLocation().toURI());
			name = jarFile.getName();
		} catch (URISyntaxException ex) {
			//who cares
		}

		helpFormatter.printHelp("java -jar " + name, null, options, null, true);
	}

	private Options loadOptions() {
		Options options = new Options();

		OptionBuilder.withLongOpt(OPT_NOVIS);
		OptionBuilder.withDescription("Disables 2D visualization");
		Option novis = OptionBuilder.create();
		options.addOption(novis);

		OptionBuilder.withLongOpt(OPT_NOVIS3D);
		OptionBuilder.withDescription("Disables 3D visualization");
		Option novis3d = OptionBuilder.create();
		options.addOption(novis3d);

		OptionBuilder.withLongOpt(OPT_CONF);
		OptionBuilder.hasArg();
		OptionBuilder.withArgName("Class name");
		OptionBuilder.withDescription("Name of class implementing VillageConfig interface");
		Option conf = OptionBuilder.create();
		options.addOption(conf);

		OptionBuilder.withLongOpt(OPT_HELP);
		OptionBuilder.withDescription("Prints this message");
		Option help = OptionBuilder.create();
		options.addOption(help);

		return options;
	}

	@Override
	public void create() {
		createCommons();

		createSimulation();
		createEnvironment();
		createVisualization();
		createEntities();
		createOverlayVisualization();
		create3dVisualization();

		start();
	}

	private void createCommons() {
		LOGGER.info(">>> UNIVERSE CREATION");
		universe = new PAHUniverse(villageConfig);

		if (loadVis) {
			LOGGER.info(">>> VISUALIZATION MANAGER CREATION");
			VisManager.setInitParam("Alite Tactical Environment Operator", 1024, 768);
			VisManager.init();
			VisManager.setSceneParam(villageConfig.getSceneParams());
		}

		if (loadVis3D) {
			LOGGER.info(">>> 3d VISUALIZATION MANAGER CREATION");
			Vis3dManager.setInitParam("Alite Tactical Environment World", 1024, 768);
			Vis3dManager.init();
			java.util.logging.Logger.getLogger("com.jme3").setLevel(java.util.logging.Level.SEVERE);
			Vis3dManager.setSceneParam(villageConfig.get3dSceneParams());
		}
	}

	protected void createSimulation() {
		LOGGER.info(">>> SIMULATION CREATION");
		Simulation simulation = new Simulation();
		simulation.setPrintouts(10000);
		simulation.setSimulationSpeed(0.2);
		simulation.setRunning(true); // set to false to pause the simulation as start
		universe.setSimulation(simulation);
	}

	protected void createEnvironment() {
		LOGGER.info(">>> ENVIRONMENT CREATION");
		TacticalEnvironment environment = new TacticalEnvironment(universe);
		environment.setRandom(new Random(RANDOM_SEED));
		universe.setEnvironment(environment);
	}

	abstract protected void createEntities();

	protected void addCarAgent(Point initialLocation, Set<Point> checkpoints) {
		PAHCarAgent carAgent = new StudentPAHCarAgent("car" + carCounter, universe.getMap(), universe.getPhysics().getTerrain(), universe.getEnvironment().handler(), checkpoints);
		universe.addCustomCarEntity("car" + carCounter, carAgent, Team.ALLY, initialLocation);
		carCounter++;
	}

	protected void createVisualization() {
		LOGGER.info(">>> VISUALIZATION CREATION");

		// background
		VisManager.registerLayer(ColorLayer.create(Color.GRAY));

		// static
		// VisManager.registerLayer(GraphLayer.create(universe.getMap().getStreetGraph(), Color.BLACK, Color.BLUE, 1, 5));
		// VisManager.registerLayer(CtolManeuverGraphLayer.create(universe.getMap().getAirplaneManeuverGraph(), Color.RED, Color.YELLOW, 1, 5, "a"));
		VisManager.registerLayer(BuildingLayer.create(universe.getMap(), Color.WHITE, 2));
		// VisManager.registerLayer(MarkerLayer.create(universe.getMap(), Color.ORANGE, "m"));
		// VisManager.registerLayer(MapPositionIdLayer.create(universe.getMap(), Color.ORANGE, 1, "n"));
		VisManager.registerLayer(ZoneLayer.create());

		// dynamic
		VisManager.registerLayer(CarLayer.create(universe.getEnvironment().getCarStorage()));

		VisManager.registerLayer(BubbleLayer.create());
	}

	protected void createOverlayVisualization() {
		LOGGER.info(">>> OVERLAY VISUALIZATION CREATION");

		VisManager.registerLayer(SimulationControlLayer.create(universe.getSimulation()));
		VisManager.registerLayer(FpsLayer.create());
		VisManager.registerLayer(VisInfoLayer.create());
		VisManager.registerLayer(LogoLayer.create(ClassLoader.getSystemResource(LOGO_IMAGE_FILE)));
		VisManager.registerLayer(HelpLayer.create());
	}

	protected void create3dVisualization() {
		LOGGER.info(">>> 3d VISUALIZATION CREATION");

		//Vis3dManager.registerLayer(Logo3dLayer.create(LOGO_IMAGE_FILE, new Vector2f(128, 64), new Vector2f(20, 0)));

		Vis3dManager.registerLayer(Skybox3dLayer.create(universe.getVillageConfig().getSkyboxScale(), universe.getVillageConfig().getSunRotation()));
		Vis3dManager.registerLayer(Village3dLayer.create(universe.getVillageConfig()));

		Vis3dManager.registerLayer(Mdars3dLayer.create(universe.getEnvironment().getCarStorage(), universe.getPhysics()));
		Vis3dManager.registerLayer(Skeldar3dLayer.create(universe.getEnvironment().getHelicopterStorage(), VisibilityConeType.DOWN));
		Vis3dManager.registerLayer(Person3dLayer.create(universe.getEnvironment().getPersonStorage(), universe));
		Vis3dManager.registerLayer(TacticalPerson3dLayer.create(universe.getEnvironment().getTacticalPersonStorage(), universe, Team.ALLY, Team.FOE));
		Vis3dManager.registerLayer(Vidar3dLayer.create(universe.getEnvironment().getQuadrotorStorage(), universe.getPhysics()));
		Vis3dManager.registerLayer(Procerus3dLayer.create(universe.getEnvironment().getAirplaneStorage(), VisibilityConeType.FRONT));

		Vis3dManager.registerLayer(SimulationControl3dLayer.create(universe.getSimulation()));
	}

	private void start() {
		LOGGER.info(">>> UNIVERSE START");
		universe.start();
	}

	public void enableVis2d(boolean enable) {
		this.loadVis = enable;
	}

	public void enableVis3d(boolean enable) {
		this.loadVis3D = enable;
	}

	protected double getAltitude(double x, double y) {
		return universe.getPhysics().getAltitude((float) x, (float) y);
	}
}
