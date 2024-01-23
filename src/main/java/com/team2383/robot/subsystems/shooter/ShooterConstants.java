package com.team2383.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ShooterConstants {
    public static int kTopMotorID = 6;
    public static int kBottomMotorID = 7;

    public static Slot0Configs kBottomConfigs = new Slot0Configs()
            .withKP(0.08)
            .withKI(0)
            .withKD(0)
            .withKA(0.0055439)
            .withKV(0.11381)
            .withKS(0.21147);

    public static Slot0Configs kTopConfigs = new Slot0Configs()
            .withKP(0.086154)
            .withKI(0)
            .withKD(0)
            .withKA(0.021305)
            .withKV(0.11179)
            .withKS(0.03017);

    public static int kSideMotorID = 8;

    public static double kSideP = 0.00011;
    public static double kSideI = 0.0;
    public static double kSideD = 0.0;
    public static double kSideV = 0.0020796;
    public static double kSideS = 0.14016;

}
