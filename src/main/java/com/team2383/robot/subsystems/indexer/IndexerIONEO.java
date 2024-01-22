package com.team2383.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;

public class IndexerIONEO implements IndexerIO {
    private final CANSparkMax m_indexer;

    public IndexerIONEO() {
        m_indexer = new CANSparkMax(IndexerConstants.kIndexerID, CANSparkMax.MotorType.kBrushless);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.power = m_indexer.get();
    }

    @Override
    public void setPower(double power) {
        m_indexer.set(power);
    }
}
