package com.team2383.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;

public class ShooterConstants {
    public static int kTopMotorID = 9;
    public static int kBottomMotorID = 10;

    public static Slot0Configs kBottomConfigs = new Slot0Configs()
            .withKP(0.25)
            .withKI(0)
            .withKD(0)
            .withKA(0)
            .withKV(0.2025375)
            .withKS(0.01);

    public static Slot1Configs kBottomConfigsHigh = new Slot1Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0)
            .withKA(0)
            .withKV(0.175)
            .withKS(0.01);

    public static Slot0Configs kTopConfigs = new Slot0Configs()
            .withKP(0.25)
            .withKI(0)
            .withKD(0)
            .withKA(0)
            .withKV(0.2025375)
            .withKS(0.01);

    public static Slot1Configs kTopConfigsHigh = new Slot1Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0)
            .withKA(0)
            .withKV(0.17)
            .withKS(0.01);

    public static int kSideMotorID = 11;

    public static double kSideP = 0.0001;
    public static double kSideI = 0.0000001;
    public static double kSideD = 0.0;
    public static double kSideV = 0.0;
    public static double kSideS = 0.0;

    public static record ShooterGains(double kP, double kI, double kD, double kA, double kV, double kS) {
    }
}
