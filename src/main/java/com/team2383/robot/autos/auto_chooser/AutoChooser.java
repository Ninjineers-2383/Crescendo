package com.team2383.robot.autos.auto_chooser;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.team2383.robot.autos.OnePieceAuto;
import com.team2383.robot.autos.ThreePieceAuto;
import com.team2383.robot.autos.TwoPieceAuto;
import com.team2383.robot.autos.auto_chooser.AutoQuestionResponses.QuestionResponses;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser {
    private final LoggedDashboardChooser<String> autoRoutineChooser;

    private LoggedDashboardChooser<QuestionResponses> question1;
    private LoggedDashboardChooser<QuestionResponses> question2;
    private LoggedDashboardChooser<QuestionResponses> question3;
    private LoggedDashboardChooser<QuestionResponses> question4;
    private LoggedDashboardChooser<QuestionResponses> question5;
    private LoggedDashboardChooser<QuestionResponses> question6;
    private LoggedDashboardChooser<QuestionResponses> question7;
    private LoggedDashboardChooser<QuestionResponses> question8;
    private LoggedDashboardChooser<QuestionResponses> question9;
    private LoggedDashboardChooser<QuestionResponses> question10;
    private LoggedDashboardChooser<QuestionResponses> question11;

    private final DrivetrainSubsystem m_drive;
    private final ElevatorSubsystem m_elevator;
    private final WristSubsystem m_wrist;
    private final FeederSubsystem m_feeder;

    int i = 0;

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
        question6 = new LoggedDashboardChooser<QuestionResponses>("Question 6");
        question7 = new LoggedDashboardChooser<QuestionResponses>("Question 7");
        question8 = new LoggedDashboardChooser<QuestionResponses>("Question 8");
        question9 = new LoggedDashboardChooser<QuestionResponses>("Question 9");
        question10 = new LoggedDashboardChooser<QuestionResponses>("Question 10");
        question11 = new LoggedDashboardChooser<QuestionResponses>("Question 11");

        autoRoutineChooser.addDefaultOption("No Auto :(", "No Auto");
        autoRoutineChooser.addOption("One Piece Auto", "One Piece Auto");
        autoRoutineChooser.addOption("Two Piece Auto", "Two Piece Auto");
        autoRoutineChooser.addOption("Three Piece Auto", "Three Piece Auto");
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
                question3.addOption("Mobility Charge", QuestionResponses.MOBILITYCHARGE);

                if (question3.get() != null) {
                    switch (question3.get()) {
                        case MOBILITYCLEAN:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONECLEAN1);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCUBECLEAN1);
                            question5.addDefaultOption("Engage", QuestionResponses.ENGAGEOUTSIDE);
                            question5.addOption("Stop", QuestionResponses.STOP);
                            break;
                        case MOBILITYDIRTY:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONEDIRTY1);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCUBEDIRTY1);
                            question5.addDefaultOption("Engage", QuestionResponses.ENGAGEOUTSIDE);
                            question5.addOption("Stop", QuestionResponses.STOP);
                            break;
                        case MOBILITYCHARGE:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONECHARGE1);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCONECHARGE1);
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
                question6.addDefaultOption("N/A", QuestionResponses.NA);
                question7.addDefaultOption("N/A", QuestionResponses.NA);
                question8.addDefaultOption("N/A", QuestionResponses.NA);
                question9.addDefaultOption("N/A", QuestionResponses.NA);
                question10.addDefaultOption("N/A", QuestionResponses.NA);
                question11.addDefaultOption("N/A", QuestionResponses.NA);
                break;
            case "Two Piece Auto":
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

                question3.addDefaultOption("Mobility Clean", QuestionResponses.MOBILITYCLEAN);
                question3.addOption("Mobility Dirty", QuestionResponses.MOBILITYDIRTY);
                question3.addOption("Mobility Charge", QuestionResponses.MOBILITYCHARGE);

                if (question3.get() != null) {
                    switch (question3.get()) {
                        case MOBILITYCLEAN:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONECLEAN1);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCUBECLEAN1);
                            break;
                        case MOBILITYDIRTY:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONEDIRTY1);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCUBEDIRTY1);
                            break;
                        case MOBILITYCHARGE:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONECHARGE1);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCONECHARGE1);
                        default:
                            break;
                    }
                }

                question5.addDefaultOption("Cone1", QuestionResponses.CONE1);
                question5.addOption("Cube1", QuestionResponses.CUBE1);
                question5.addOption("Cone2", QuestionResponses.CONE2);
                question5.addOption("Cone3", QuestionResponses.CONE3);
                question5.addOption("Cube2", QuestionResponses.CUBE2);
                question5.addOption("Cone4", QuestionResponses.CONE4);
                question5.addOption("Cone5", QuestionResponses.CONE5);
                question5.addOption("Cube3", QuestionResponses.CUBE3);
                question5.addOption("Cone6", QuestionResponses.CONE6);

                question6.addDefaultOption("High", QuestionResponses.HIGH);
                question6.addOption("Medium", QuestionResponses.MEDIUM);
                question6.addOption("Hybrid", QuestionResponses.HYBRID);

                question7.addDefaultOption("Engage", QuestionResponses.ENGAGECOMMUNITY);
                question7.addOption("Stop", QuestionResponses.STOP);

                question8.addDefaultOption("N/A", QuestionResponses.NA);
                question9.addDefaultOption("N/A", QuestionResponses.NA);
                question10.addDefaultOption("N/A", QuestionResponses.NA);
                question11.addDefaultOption("N/A", QuestionResponses.NA);
                break;
            case "Three Piece Auto":
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

                question3.addDefaultOption("Mobility Clean", QuestionResponses.MOBILITYCLEAN);
                question3.addOption("Mobility Dirty", QuestionResponses.MOBILITYDIRTY);
                question3.addOption("Mobility Charge", QuestionResponses.MOBILITYCHARGE);

                if (question3.get() != null) {
                    switch (question3.get()) {
                        case MOBILITYCLEAN:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONECLEAN1);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCUBECLEAN1);
                            break;
                        case MOBILITYDIRTY:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONEDIRTY1);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCUBEDIRTY1);
                            break;
                        case MOBILITYCHARGE:
                            question4.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONECHARGE1);
                            question4.addOption("Feed Cube", QuestionResponses.FEEDCONECHARGE1);
                        default:
                            break;
                    }
                }

                question5.addDefaultOption("Cone1", QuestionResponses.CONE1);
                question5.addOption("Cube1", QuestionResponses.CUBE1);
                question5.addOption("Cone2", QuestionResponses.CONE2);
                question5.addOption("Cone3", QuestionResponses.CONE3);
                question5.addOption("Cube2", QuestionResponses.CUBE2);
                question5.addOption("Cone4", QuestionResponses.CONE4);
                question5.addOption("Cone5", QuestionResponses.CONE5);
                question5.addOption("Cube3", QuestionResponses.CUBE3);
                question5.addOption("Cone6", QuestionResponses.CONE6);

                question6.addDefaultOption("High", QuestionResponses.HIGH);
                question6.addOption("Medium", QuestionResponses.MEDIUM);
                question6.addOption("Hybrid", QuestionResponses.HYBRID);

                question7.addDefaultOption("Mobility Clean", QuestionResponses.MOBILITYCLEAN);
                question7.addOption("Mobility Dirty", QuestionResponses.MOBILITYDIRTY);
                question7.addOption("Mobility Charge", QuestionResponses.MOBILITYCHARGE);

                if (question7.get() != null) {
                    switch (question7.get()) {
                        case MOBILITYCLEAN:
                            question8.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONECLEAN2);
                            question8.addOption("Feed Cube", QuestionResponses.FEEDCUBECLEAN2);
                            break;
                        case MOBILITYDIRTY:
                            question8.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONEDIRTY2);
                            question8.addOption("Feed Cube", QuestionResponses.FEEDCUBEDIRTY2);
                            break;
                        case MOBILITYCHARGE:
                            question8.addDefaultOption("Feed Cone", QuestionResponses.FEEDCONECHARGE2);
                            question8.addOption("Feed Cube", QuestionResponses.FEEDCONECHARGE2);
                        default:
                            break;
                    }
                }

                question9.addDefaultOption("Cone1", QuestionResponses.CONE1);
                question9.addOption("Cube1", QuestionResponses.CUBE1);
                question9.addOption("Cone2", QuestionResponses.CONE2);
                question9.addOption("Cone3", QuestionResponses.CONE3);
                question9.addOption("Cube2", QuestionResponses.CUBE2);
                question9.addOption("Cone4", QuestionResponses.CONE4);
                question9.addOption("Cone5", QuestionResponses.CONE5);
                question9.addOption("Cube3", QuestionResponses.CUBE3);
                question9.addOption("Cone6", QuestionResponses.CONE6);

                question10.addDefaultOption("High", QuestionResponses.HIGH);
                question10.addOption("Medium", QuestionResponses.MEDIUM);
                question10.addOption("Hybrid", QuestionResponses.HYBRID);

                question11.addDefaultOption("Engage", QuestionResponses.ENGAGECOMMUNITY);
                question11.addOption("Stop", QuestionResponses.STOP);
                break;
            case "No Auto":
                question1.addDefaultOption("N/A", QuestionResponses.NA);
                question2.addDefaultOption("N/A", QuestionResponses.NA);
                question3.addDefaultOption("N/A", QuestionResponses.NA);
                question4.addDefaultOption("N/A", QuestionResponses.NA);
                question5.addDefaultOption("N/A", QuestionResponses.NA);
                question6.addDefaultOption("N/A", QuestionResponses.NA);
                question7.addDefaultOption("N/A", QuestionResponses.NA);
                question8.addDefaultOption("N/A", QuestionResponses.NA);
                question9.addDefaultOption("N/A", QuestionResponses.NA);
                question10.addDefaultOption("N/A", QuestionResponses.NA);
                question11.addDefaultOption("N/A", QuestionResponses.NA);
                break;
        }
    }

    public QuestionResponses[] getResponses() {
        return new QuestionResponses[] { question1.get(), question2.get(), question3.get(), question4.get(),
                question5.get(), question6.get(), question7.get(), question8.get(), question9.get(), question10.get(),
                question11.get() };
    }

    public Command getAutonomousCommand() {
        Command nullAuto = null;
        switch (autoRoutineChooser.get()) {
            case "One Piece Auto":
                return new OnePieceAuto(m_drive, m_elevator, m_wrist, m_feeder, getResponses());
            case "Two Piece Auto":
                return new TwoPieceAuto(m_drive, m_elevator, m_wrist, m_feeder, getResponses());
            case "Three Piece Auto":
                return new ThreePieceAuto(m_drive, m_elevator, m_wrist, m_feeder, getResponses());
            default:
                return nullAuto;
        }

    }

    public void clearAll() {
        question1 = new LoggedDashboardChooser<QuestionResponses>("Question 1");
        question2 = new LoggedDashboardChooser<QuestionResponses>("Question 2");
        question3 = new LoggedDashboardChooser<QuestionResponses>("Question 3");
        question4 = new LoggedDashboardChooser<QuestionResponses>("Question 4");
        question5 = new LoggedDashboardChooser<QuestionResponses>("Question 5");
        question6 = new LoggedDashboardChooser<QuestionResponses>("Question 6");
        question7 = new LoggedDashboardChooser<QuestionResponses>("Question 7");
        question8 = new LoggedDashboardChooser<QuestionResponses>("Question 8");
        question9 = new LoggedDashboardChooser<QuestionResponses>("Question 9");
        question10 = new LoggedDashboardChooser<QuestionResponses>("Question 10");
        question11 = new LoggedDashboardChooser<QuestionResponses>("Question 11");
    }
}
