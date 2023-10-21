// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Note: Translations calculated with the formula graphed here
 * https://www.desmos.com/calculator/jj9i5bdjhe
 * <p>
 * The circle function is used in
 * {@link com.team2383.robot.commands.JoystickDriveCommand#getCenterOfRotation
 * JoystickDriveCommand}
 */

public final class Constants {
    public static final String kRIOBus = "rio";
    public static final String kCANivoreBus = "Drive";

    public static final RobotType robot = RobotType.ROBOT_COMP;

    public static RobotType getRobot() {
        if (RobotBase.isReal()) {
            if (robot == RobotType.ROBOT_SIM) { // Invalid robot selected
                return RobotType.ROBOT_COMP;
            } else {
                return robot;
            }
        } else {
            return robot;
        }
    }

    public static Mode getMode() {
        switch (getRobot()) {
            case ROBOT_COMP:
                return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

            case ROBOT_SIM:
                return Mode.SIM;

            default:
                return Mode.REAL;
        }
    }

    public static enum RobotType {
        ROBOT_COMP, ROBOT_SIM
    }

    public static enum Mode {
        REAL, REPLAY, SIM
    }

    public static final class OI {
        // Axis
        public static int DriveX = Robot.isReal() ? 5 : 0;
        public static int DriveY = Robot.isReal() ? 4 : 1;
        public static int DriveOmega = Robot.isReal() ? 0 : 2;
        public static int IntakeIn = Robot.isReal() ? 3 : 3;
        public static int IntakeOut = Robot.isReal() ? 2 : 4;

        // Buttons
        public static int FieldCentric = Robot.isReal() ? 6 : 1;
        public static int ResetHeading = Robot.isReal() ? 8 : 2;
    }
}
