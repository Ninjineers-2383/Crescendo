package com.team2383.robot.autos;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.team2383.robot.autos.AutoQuestionResponses.QuestionResponses;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser {
    private final LoggedDashboardChooser<String> autoRoutineChooser;

    private final LoggedDashboardChooser<QuestionResponses> question1;
    private final LoggedDashboardChooser<QuestionResponses> question2;
    private final LoggedDashboardChooser<QuestionResponses> question3;
    private final LoggedDashboardChooser<QuestionResponses> question4;
    private final LoggedDashboardChooser<QuestionResponses> question5;

    private final DrivetrainSubsystem m_drive;
    private final ElevatorSubsystem m_elevator;
    private final WristSubsystem m_wrist;
    private final FeederSubsystem m_feeder;

    public AutoChooser(DrivetrainSubsystem drive, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder) {
        m_drive = drive;
        m_elevator = elevator;
        m_wrist = wrist;
        m_feeder = feeder;

        autoRoutineChooser = new LoggedDashboardChooser<String>("Auto");
        question1 = new LoggedDashboardChooser<QuestionResponses>("Question 1");
        question2 = new LoggedDashboardChooser<QuestionResponses>("Question 2");
        question3 = new LoggedDashboardChooser<QuestionResponses>("Question 3");
        question4 = new LoggedDashboardChooser<QuestionResponses>("Question 4");
        question5 = new LoggedDashboardChooser<QuestionResponses>("Question 5");

        autoRoutineChooser.addDefaultOption("No Auto :(", "No Auto");
        autoRoutineChooser.addOption("One Piece Auto", "One Piece Auto");
    }

    public void periodic() {
        switch (autoRoutineChooser.get()) {
            case "One Piece Auto":
                question1.addDefaultOption("Cone1", QuestionResponses.CONE1);
                question1.addOption("Cube1", QuestionResponses.CUBE1);
                question1.addOption("Cone2", QuestionResponses.CONE2);
                question1.addOption("Cone3", QuestionResponses.CONE3);
                question1.addOption("Cube2", QuestionResponses.CUBE2);
                question1.addOption("Cone4", QuestionResponses.CONE4);
                question1.addOption("Cone5", QuestionResponses.CONE5);
                question1.addOption("Cube3", QuestionResponses.CUBE3);
                question1.addOption("Cone6", QuestionResponses.CONE6);

                question2.addDefaultOption("High", QuestionResponses.HIGH);
                question2.addOption("Medium", QuestionResponses.MEDIUM);
                question2.addOption("Hybrid", QuestionResponses.HYBRID);

                question3.addDefaultOption("Engage", QuestionResponses.ENGAGECOMMUNITY);
                question3.addOption("Stop", QuestionResponses.STOP);
                question3.addOption("Mobility Clean", QuestionResponses.MOBILITYCLEAN);
                question3.addOption("Mobility Dirty", QuestionResponses.MOBILITYDIRTY);

                question4.addDefaultOption("N/A", QuestionResponses.NA);
                question5.addDefaultOption("N/A", QuestionResponses.NA);

                if (question3.get() != null) {
                    switch (question3.get()) {
                        case MOBILITYCLEAN:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONECLEAN);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCUBECLEAN);
                            question5.addDefaultOption("Engage", QuestionResponses.ENGAGEOUTSIDE);
                            question5.addOption("Stop", QuestionResponses.STOP);
                            break;
                        case MOBILITYDIRTY:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONEDIRTY);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCUBEDIRTY);
                            question5.addDefaultOption("Engage", QuestionResponses.ENGAGEOUTSIDE);
                            question5.addOption("Stop", QuestionResponses.STOP);
                            break;
                        case ENGAGECOMMUNITY:
                            question4.addDefaultOption("N/A", QuestionResponses.NA);
                            question5.addDefaultOption("N/A", QuestionResponses.NA);
                        default:
                            break;
                    }
                }

                break;
            default:

                break;

        }
    }

    public QuestionResponses[] getResponses() {
        return new QuestionResponses[] { question1.get(), question2.get(), question3.get(), question4.get(),
                question5.get() };
    }

    public Command getAutonomousCommand() {
        Command nullAuto = null;
        switch (autoRoutineChooser.get()) {
            case "One Piece Auto":
                return new OnePieceAuto(m_drive, m_elevator, m_wrist, m_feeder, getResponses());
            default:
                return nullAuto;
        }

    }
}
