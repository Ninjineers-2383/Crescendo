package com.team2383.robot.commands.blizzard;

import com.team2383.robot.autos.auto_chooser.AutoQuestionResponses.QuestionResponses;

import edu.wpi.first.math.geometry.Rotation2d;

public class BlizzardPresets {
    public static final BlizzardPosition ZERO = new BlizzardPosition(0,
            Rotation2d.fromDegrees(90));

    public static final BlizzardPosition GROUND_INTAKE = new BlizzardPosition(0.08,
            Rotation2d.fromRadians(0.25));

    public static final BlizzardPosition CONE_CHUTE = new BlizzardPosition(0.06,
            Rotation2d.fromRadians(1.32));

    public static final BlizzardPosition MIDDLE = new BlizzardPosition(0.8,
            Rotation2d.fromRadians(0.605));

    public static final BlizzardPosition HIGH = new BlizzardPosition(1.33,
            Rotation2d.fromRadians(0.73));

    public static final BlizzardPosition HIGH_2 = new BlizzardPosition(1.33,
            Rotation2d.fromRadians(0.93));

    public static final BlizzardPosition SLIDER = new BlizzardPosition(0.55,
            Rotation2d.fromRadians(1.46));

    public static BlizzardPosition responseToPreset(QuestionResponses response) {
        switch (response) {
            case HYBRID:
                return CONE_CHUTE;
            case MEDIUM:
                return MIDDLE;
            case HIGH:
                return HIGH;
            default:
                return null;
        }
    }
}
