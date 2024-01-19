package com.team2383.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ShooterConstants {
    public static int kTopMotorID = 6;
    public static int kBottomMotorID = 7;

    public static int kSideMotorID = 8;

    public static Slot0Configs kTopConfigs = new Slot0Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0)
            .withKA(0.05)
            .withKV(0.05)
            .withKS(0.05);

    public static Slot0Configs kBottomConfigs = new Slot0Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0)
            .withKA(0.05)
            .withKV(0.05)
            .withKS(0.05);

    public static double kBottomP = 1.0;
    public static double kBottomI = 0.0;
    public static double kBottomD = 0.0;
    public static double kBottomF = 0.0;

}
