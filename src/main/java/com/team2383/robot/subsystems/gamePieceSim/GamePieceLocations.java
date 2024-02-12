package com.team2383.robot.subsystems.gamePieceSim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class GamePieceLocations {
    public static double fieldLength = 16.54175;
    public static double fieldWidth = 8.21055;

    public static double shooterRadiusMeters = Units.inchesToMeters(3);

    public static double coefficientOfDrag = 0.1;

    public static double noteHeight = Units.inchesToMeters(1);

    public static Pose3d note1 = new Pose3d(2.8930346, fieldWidth - 1.2100052, noteHeight,
            new Rotation3d());

    public static Pose3d note2 = new Pose3d(2.8930346, fieldWidth - 2.6578052, noteHeight,
            new Rotation3d());

    public static Pose3d note3 = new Pose3d(2.8930346, 4.0987472, noteHeight,
            new Rotation3d());

    public static Pose3d note4 = new Pose3d(8.2677, 0.7528052, noteHeight, new Rotation3d());

    public static Pose3d note5 = new Pose3d(8.2677, 2.4292052, noteHeight, new Rotation3d());

    public static Pose3d note6 = new Pose3d(8.2677, 4.1056052, noteHeight, new Rotation3d());

    public static Pose3d note7 = new Pose3d(8.2677, 5.7820052, noteHeight, new Rotation3d());

    public static Pose3d note8 = new Pose3d(8.2677, 7.4584052, noteHeight, new Rotation3d());

    public static Pose3d note9 = new Pose3d(13.6423654, fieldWidth - 1.1413998, noteHeight,
            new Rotation3d());

    public static Pose3d note10 = new Pose3d(13.6423654, fieldWidth - 2.6578052, noteHeight,
            new Rotation3d());

    public static Pose3d note11 = new Pose3d(13.6423654, 4.0987472, noteHeight, new Rotation3d());

    public static Pose3d[] notes = { note1, note2, note3, note4, note5, note6, note7, note8, note9, note10,
            note11 };
}
