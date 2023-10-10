package team2383.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.team2383.robot.subsystems.feeder.FeederIOSim;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;

public class FeederTest {
    static final double DELTA = 1e-2; // acceptable deviation range
    public FeederSubsystem m_feeder;

    @BeforeAll
    public static void before() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        CommandScheduler.getInstance().enable(); // enable the command scheduler
    }

    @BeforeEach // this method will run before each test
    public void setup() {
        m_feeder = new FeederSubsystem(new FeederIOSim()); // create a new intake object
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach // this method will run after each test
    public void shutdown() throws Exception {
        CommandScheduler.getInstance().close();
    }

    @Test // marks this method as a test
    public void setPowerWorks() {
        m_feeder.setPower(0.5); // set the power to 0.5
        CommandScheduler.getInstance().run();
        assertEquals(0.5, m_feeder.getPower(), DELTA); // assert that the power is 0.5
    }
}