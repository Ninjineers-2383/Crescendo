package com.team2383.robot.commands.subsystem.trap_arm.tuning;

import com.team2383.lib.controller.TunableArmFeedforward;
import com.team2383.robot.subsystems.trap_arm.TrapArmConstants;
import com.team2383.robot.subsystems.trap_arm.TrapArmSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TrapArmTuningCommand extends Command {
    private final TrapArmSubsystem trapArm;
    private TunableArmFeedforward feedforward;
    private PIDController pid;

    public TrapArmTuningCommand(TrapArmSubsystem trapArm) {
        this.trapArm = trapArm;

        this.feedforward = TrapArmConstants.kFeedforwardController;
        this.pid = TrapArmConstants.kFeedbackController;
    }

    @Override
    public void initialize() {
        SmartDashboard.putData("Trap Arm PID", pid);
        SmartDashboard.putData("Trap Arm Feedforward", feedforward);
    }

    @Override
    public void execute() {
        trapArm.setFeedforward(feedforward);
        trapArm.setPIDController(pid);
    }
}
