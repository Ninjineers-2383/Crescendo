package com.team2383.robot.subsystems.feeder;

import com.team2383.robot.autos.auto_chooser.AutoQuestionResponses.QuestionResponses;

public class FeederConstants {
    public static int kMotorID = 6;

    public static boolean responseToGamePiece(QuestionResponses response) {
        switch (response) {
            case CONE1:
            case CONE2:
            case CONE3:
            case CONE4:
            case CONE5:
            case CONE6:
                return true;
            case CUBE1:
            case CUBE2:
            case CUBE3:
                return false;
            default:
                return false;
        }
    }
}
