package com.team2383.robot.commands.blizzard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class BlizzardPresets {
    public static final BlizzardPosition GROUND_INTAKE = new BlizzardPosition(0.08,
            Rotation2d.fromRadians(-0.16));

    public static final BlizzardPosition CONE_CHUTE = new BlizzardPosition(0.06,
            Rotation2d.fromRadians(1.32));

    public static final BlizzardPosition MIDDLE = new BlizzardPosition(0.75,
            Rotation2d.fromRadians(0.605));

    public static final BlizzardPosition HIGH = new BlizzardPosition(1.33,
            Rotation2d.fromRadians(0.73));

    public static final BlizzardPosition SLIDER = new BlizzardPosition(0.47,
            Rotation2d.fromRadians(1.46));

}
