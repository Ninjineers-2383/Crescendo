package com.team2383.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ShooterConstants {
    public static int kTopMotorID = 9;
    public static int kBottomMotorID = 10;

    public static Slot0Configs kBottomConfigs = new Slot0Configs()
            .withKP(0.5)
            .withKI(0)
            .withKD(0)
            .withKA(0)
            .withKV(0.1455)
            .withKS(0);

    public static Slot0Configs kTopConfigs = new Slot0Configs()
            .withKP(0.5)
            .withKI(0)
            .withKD(0)
            .withKA(0)
            .withKV(0.1455)
            .withKS(0);

    public static int kSideMotorID = 11;

    public static double kSideP = 0.0001;
    public static double kSideI = 0.0000001;
    public static double kSideD = 0.0;
    public static double kSideV = 0.0;
    public static double kSideS = 0.0;

}
