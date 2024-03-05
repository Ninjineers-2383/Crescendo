package com.team2383.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class IndexerIONEO implements IndexerIO {
    private final CANSparkMax m_indexer;

    public IndexerIONEO() {
        m_indexer = new CANSparkMax(IndexerConstants.kIndexerID, CANSparkMax.MotorType.kBrushless);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.motorConnected = m_indexer.getLastError() == REVLibError.kOk;

        inputs.power = m_indexer.get();
    }

    @Override
    public void setPower(double power) {
        m_indexer.set(power);
    }
}
