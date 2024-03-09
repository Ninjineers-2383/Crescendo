package com.team2383.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIONEO implements IndexerIO {
    private final CANSparkMax m_indexer;

    private final DigitalInput m_beamBreak;

    public IndexerIONEO() {
        m_indexer = new CANSparkMax(IndexerConstants.kIndexerID, CANSparkMax.MotorType.kBrushless);
        m_indexer.restoreFactoryDefaults();
        m_indexer.setInverted(true);
        m_beamBreak = new DigitalInput(0);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.motorConnected = m_indexer.getLastError() == REVLibError.kOk;

        inputs.power = m_indexer.get();
        inputs.supplyVoltage = m_indexer.getAppliedOutput();
        inputs.supplyCurrent = m_indexer.getOutputCurrent();

        inputs.beamBreakTripped = !m_beamBreak.get();
    }

    @Override
    public void setPower(double power) {
        m_indexer.set(power);
    }
}
