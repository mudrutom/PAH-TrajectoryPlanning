package cz.agents.alite.pahtactical.vis;

import cz.agents.alite.tactical.util.Point;
import cz.agents.alite.vis.element.Line;
import cz.agents.alite.vis.element.aggregation.LineElements;
import cz.agents.alite.vis.element.implemetation.LineImpl;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.CommonLayer;
import cz.agents.alite.vis.layer.terminal.LineLayer;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;


public class PlanLayer extends CommonLayer {

	public static VisLayer create(final PlanProvider provider, final Color color, final int strokeWidth) {
		GroupLayer toggle = GroupLayer.create();

		toggle.addSubLayer(LineLayer.create(new LineElements() {

			@Override
			public Iterable<? extends Line> getLines() {
				ArrayList<Line> lines = new ArrayList<Line>();

				List<Point> plan = provider.getPlan();

				if (plan != null) {
					Point oldPosition = null;
					for (Point position : plan) {
						if (oldPosition != null) {
							LineImpl lineImpl = new LineImpl(oldPosition, position);
							lines.add(lineImpl);
						}
						oldPosition = position;
					}
				}
				return lines;
			}

			@Override
			public Color getColor() {
				return color;
			}

			@Override
			public int getStrokeWidth() {
				return strokeWidth;
			}
		}));

		return toggle;
	}

	public static interface PlanProvider {
		public List<Point> getPlan();
	}
}

