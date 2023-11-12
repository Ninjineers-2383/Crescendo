package com.team2383.robot.autos.auto_chooser;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.team2383.robot.autos.auto_chooser.AutoQuestionResponses.QuestionResponses;

public class TeleOpChooser {
    private final LoggedDashboardChooser<QuestionResponses> teleOpRoutineChooser;
    private final LoggedDashboardChooser<QuestionResponses> heightChooser;

    public TeleOpChooser() {
        teleOpRoutineChooser = new LoggedDashboardChooser<QuestionResponses>("TeleOp Scoring");

        teleOpRoutineChooser.addDefaultOption("Cone1", QuestionResponses.CONE1);
        teleOpRoutineChooser.addOption("Cube1", QuestionResponses.CUBE1);
        teleOpRoutineChooser.addOption("Cone2", QuestionResponses.CONE2);
        teleOpRoutineChooser.addOption("Cone3", QuestionResponses.CONE3);
        teleOpRoutineChooser.addOption("Cube2", QuestionResponses.CUBE2);
        teleOpRoutineChooser.addOption("Cone4", QuestionResponses.CONE4);
        teleOpRoutineChooser.addOption("Cone5", QuestionResponses.CONE5);
        teleOpRoutineChooser.addOption("Cube3", QuestionResponses.CUBE3);
        teleOpRoutineChooser.addOption("Cone6", QuestionResponses.CONE6);

        heightChooser = new LoggedDashboardChooser<QuestionResponses>("TeleOp Height");

        heightChooser.addDefaultOption("High", QuestionResponses.HIGH);
        heightChooser.addOption("Middle", QuestionResponses.MEDIUM);
        heightChooser.addOption("Low", QuestionResponses.HYBRID);
    }

    public QuestionResponses[] getResponses() {
        if (teleOpRoutineChooser.get() == null) {
            return new QuestionResponses[] { QuestionResponses.CONE1, QuestionResponses.HIGH };
        }
        return new QuestionResponses[] { teleOpRoutineChooser.get(), heightChooser.get() };
    }
}
