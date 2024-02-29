package com.team2383.robot.commands.subsystem.indexer;

import com.team2383.robot.subsystems.indexer.IndexerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class IndexerCommand extends Command {
    private final IndexerSubsystem indexer;
    private final DoubleSupplier power;

    public IndexerCommand(IndexerSubsystem indexer, DoubleSupplier power) {
        this.indexer = indexer;
        this.power = power;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setPower(power.getAsDouble());
    }

    @Override
    public void execute() {
        indexer.setPower(power.getAsDouble());
    }

}
