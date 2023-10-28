package com.team2383.robot.autos;

import com.team2383.robot.FieldConstants;
import com.team2383.robot.autos.AutoQuestionResponses.QuestionResponses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoStartPoses {
    public static Pose2d CONE1 = new Pose2d(FieldConstants.Grids.outerX + 0.42, FieldConstants.Grids.nodeY[0],
            Rotation2d.fromDegrees(180));

    public static Pose2d CUBE1 = new Pose2d(FieldConstants.Grids.outerX + 0.42, FieldConstants.Grids.nodeY[1],
            Rotation2d.fromDegrees(180));

    public static Pose2d CONE2 = new Pose2d(FieldConstants.Grids.outerX + 0.42, FieldConstants.Grids.nodeY[2],
            Rotation2d.fromDegrees(180));

    public static Pose2d CONE3 = new Pose2d(FieldConstants.Grids.outerX + 0.42, FieldConstants.Grids.nodeY[3],
            Rotation2d.fromDegrees(180));

    public static Pose2d CUBE2 = new Pose2d(FieldConstants.Grids.outerX + 0.42, FieldConstants.Grids.nodeY[4],
            Rotation2d.fromDegrees(180));

    public static Pose2d CONE4 = new Pose2d(FieldConstants.Grids.outerX + 0.42, FieldConstants.Grids.nodeY[5],
            Rotation2d.fromDegrees(180));

    public static Pose2d CONE5 = new Pose2d(FieldConstants.Grids.outerX + 0.42, FieldConstants.Grids.nodeY[6],
            Rotation2d.fromDegrees(180));

    public static Pose2d CUBE3 = new Pose2d(FieldConstants.Grids.outerX + 0.42, FieldConstants.Grids.nodeY[7],
            Rotation2d.fromDegrees(180));

    public static Pose2d CONE6 = new Pose2d(FieldConstants.Grids.outerX + 0.42, FieldConstants.Grids.nodeY[8],
            Rotation2d.fromDegrees(180));

    public static Pose2d MOBILITYDIRTY = new Pose2d(5.5, 0.7, Rotation2d.fromDegrees(180));
    public static Pose2d MOBILITYCLEAN = new Pose2d(4.3, 4.8, Rotation2d.fromDegrees(180));

    public static Pose2d responseToPose(QuestionResponses response) {
        switch (response) {
            case CONE1:
                return CONE1;
            case CUBE1:
                return CUBE1;
            case CONE2:
                return CONE2;
            case CONE3:
                return CONE3;
            case CUBE2:
                return CUBE2;
            case CONE4:
                return CONE4;
            case CONE5:
                return CONE5;
            case CUBE3:
                return CUBE3;
            case CONE6:
                return CONE6;
            default:
                return null;
        }
    }

}
