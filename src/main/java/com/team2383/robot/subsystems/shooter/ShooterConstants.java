package com.team2383.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ShooterConstants {
    public static int kTopMotorID = 7;
    public static int kBottomMotorID = 8;

    public static Slot0Configs kTopConfigs = new Slot0Configs()
            .withKP(0.001)
            .withKI(0)
            .withKD(0)
            .withKA(0.05)
            .withKV(0.05)
            .withKS(0.05);

    public static Slot0Configs kBottomConfigs = new Slot0Configs()
            .withKP(0.001)
            .withKI(0)
            .withKD(0)
            .withKA(0.05)
            .withKV(0.05)
            .withKS(0.05);

    public static int kSideMotorID = 9;

    public static double kSideP = 0.001;
    public static double kSideI = 0.0;
    public static double kSideD = 0.0;
    public static double kSideA = 0.05;
    public static double kSideS = 0.0;

}
