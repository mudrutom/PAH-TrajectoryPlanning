package cz.agents.alite.pahtactical;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.tactical.creator.config.VillageConfig;
import cz.agents.alite.tactical.universe.TacticalUniverse;
import cz.agents.alite.tactical.universe.entity.embodiment.Team;
import cz.agents.alite.tactical.util.Point;

public class PAHUniverse extends TacticalUniverse {

	public PAHUniverse(VillageConfig villageConfig) {
		super(villageConfig);
	}

	public void addCustomCarEntity(String name, Entity carAgent, Team team, Point waypoint) {

		entities.put(name, carAgent);

		addCarToStorage(name, team, waypoint, null, null);
	}

}
