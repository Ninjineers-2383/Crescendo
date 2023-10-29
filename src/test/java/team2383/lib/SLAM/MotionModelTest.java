package team2383.lib.SLAM;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import com.team2383.lib.SLAM.MotionModel;

import edu.wpi.first.math.geometry.Rotation3d;

public class MotionModelTest {

    /**
     * Test the motion model when moving forward
     */
    @Test
    public void testMoveForward() {
        SimpleMatrix F_x = SimpleMatrix.identity(7);
        MotionModel m_instance = new MotionModel(F_x);

        // Robot starts at 0,0,0 facing forward
        SimpleMatrix mu = new SimpleMatrix(7, 1);
        mu.set(0, 0, 0);
        mu.set(1, 0, 0);
        mu.set(2, 0, 0);
        mu.set(3, 0, 1);
        mu.set(4, 0, 0);
        mu.set(5, 0, 0);
        mu.set(6, 0, 0);

        // Robot moves forward 1 meter
        SimpleMatrix u = new SimpleMatrix(3, 1);
        u.set(0, 0, 1);
        u.set(1, 0, 0);
        u.set(2, 0, 0);

        SimpleMatrix mu_prime = mu.plus(m_instance.apply(u, mu));

        // Robot should be at 1,0,0 facing forward
        assertEquals(mu_prime.get(0, 0), 1);
        assertEquals(mu_prime.get(1, 0), 0);
        assertEquals(mu_prime.get(2, 0), 0);
        assertEquals(mu_prime.get(3, 0), 1);
        assertEquals(mu_prime.get(4, 0), 0);
        assertEquals(mu_prime.get(5, 0), 0);
        assertEquals(mu_prime.get(6, 0), 0);
    }

    /**
     * Tests turning ccw 90 degrees
     */
    @Test
    public void clockwise90Degrees() {
        SimpleMatrix F_x = SimpleMatrix.identity(7);
        MotionModel m_instance = new MotionModel(F_x);

        // Robot starts at 0,0,0 facing forward
        SimpleMatrix mu = new SimpleMatrix(7, 1);
        mu.set(0, 0, 0);
        mu.set(1, 0, 0);
        mu.set(2, 0, 0);
        mu.set(3, 0, 1);
        mu.set(4, 0, 0);
        mu.set(5, 0, 0);
        mu.set(6, 0, 0);

        // Robot turns 90 degrees
        SimpleMatrix u = new SimpleMatrix(3, 1);
        u.set(0, 0, 0);
        u.set(1, 0, 0);
        u.set(2, 0, Math.PI / 2);

        SimpleMatrix mu_prime = mu.plus(m_instance.apply(u, mu));

        // Robot should be at 0,0,0 facing left
        Rotation3d r = new Rotation3d(0, 0, Math.PI / 2);
        assertEquals(mu_prime.get(0, 0), 0);
        assertEquals(mu_prime.get(1, 0), 0);
        assertEquals(mu_prime.get(2, 0), 0);
        assertEquals(mu_prime.get(3, 0), r.getQuaternion().getW());
        assertEquals(mu_prime.get(4, 0), r.getQuaternion().getX());
        assertEquals(mu_prime.get(5, 0), r.getQuaternion().getY());
        assertEquals(mu_prime.get(6, 0), r.getQuaternion().getZ());
    }
}
