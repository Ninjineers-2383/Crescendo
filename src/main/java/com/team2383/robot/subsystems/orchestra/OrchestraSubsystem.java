package com.team2383.robot.subsystems.orchestra;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

public class OrchestraSubsystem {
    public static OrchestraSubsystem m_instance;

    private Orchestra orchestra = new Orchestra();

    public static synchronized OrchestraSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new OrchestraSubsystem();
            return m_instance;
        }

        return m_instance;
    }

    public void addMotor(TalonFX motor) {
        orchestra.addInstrument(motor);
    }

    public void loadMusic(String filePath) {
        orchestra.loadMusic(filePath);
    }

    public void play() {
        orchestra.play();
    }

    public void pause() {
        orchestra.pause();
    }
}