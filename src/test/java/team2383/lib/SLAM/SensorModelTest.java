package team2383.lib.SLAM;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import com.team2383.lib.SLAM.SensorModelTag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class SensorModelTest {
    private static final double kTestEpsilon = 1e-12; // acceptable deviation range

    @Test
    public void testSimpleCase() {
        SensorModelTag m_instance = new SensorModelTag();

        // Robot starts at 0,0,0 facing forward
        // Tag is at 1,0,0 facing forward
        SimpleMatrix mu = new SimpleMatrix(14, 1, true,
                0, 0, 0, 1, 0, 0, 0,
                1, 0, 0, 1, 0, 0, 0);

        Pose3d robot = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        Pose3d tag = new Pose3d(1, 0, 0, new Rotation3d(0, 0, 0));

        SimpleMatrix z = m_instance.apply(mu, new SimpleMatrix(1, 1, true, 0));

        Transform3d z_true = new Transform3d(robot, tag);

        assertEquals(z.get(0, 0), z_true.getTranslation().getX());
        assertEquals(z.get(1, 0), z_true.getTranslation().getY());
        assertEquals(z.get(2, 0), z_true.getTranslation().getZ());
        assertEquals(z.get(3, 0), z_true.getRotation().getQuaternion().getW());
        assertEquals(z.get(4, 0), z_true.getRotation().getQuaternion().getX());
        assertEquals(z.get(5, 0), z_true.getRotation().getQuaternion().getY());
        assertEquals(z.get(6, 0), z_true.getRotation().getQuaternion().getZ());
    }

    @Test
    public void testComplexCase() {
        SensorModelTag m_instance = new SensorModelTag();

        Pose3d robot = new Pose3d(12, 6.2, 1.4, new Rotation3d(0, 0, 0.25));
        Pose3d tag = new Pose3d(1, 1.43, 0.234, new Rotation3d(1.4, 0.25, 0.7));

        // Robot starts at 0,0,0 facing forward
        // Tag is at 1,0,0 facing forward
        SimpleMatrix mu = new SimpleMatrix(14, 1, true,
                robot.getTranslation().getX(), robot.getTranslation().getY(), robot.getTranslation().getZ(),
                robot.getRotation().getQuaternion().getW(), robot.getRotation().getQuaternion().getX(),
                robot.getRotation().getQuaternion().getY(), robot.getRotation().getQuaternion().getZ(),
                tag.getTranslation().getX(), tag.getTranslation().getY(), tag.getTranslation().getZ(),
                tag.getRotation().getQuaternion().getW(), tag.getRotation().getQuaternion().getX(),
                tag.getRotation().getQuaternion().getY(), tag.getRotation().getQuaternion().getZ());

        SimpleMatrix z = m_instance.apply(mu, new SimpleMatrix(1, 1, true, 0));

        Transform3d z_true = new Transform3d(robot, tag);

        assertEquals(z.get(0, 0), z_true.getTranslation().getX(), kTestEpsilon);
        assertEquals(z.get(1, 0), z_true.getTranslation().getY(), kTestEpsilon);
        assertEquals(z.get(2, 0), z_true.getTranslation().getZ(), kTestEpsilon);
        assertEquals(z.get(3, 0), z_true.getRotation().getQuaternion().getW(), kTestEpsilon);
        assertEquals(z.get(4, 0), z_true.getRotation().getQuaternion().getX(), kTestEpsilon);
        assertEquals(z.get(5, 0), z_true.getRotation().getQuaternion().getY(), kTestEpsilon);
        assertEquals(z.get(6, 0), z_true.getRotation().getQuaternion().getZ(), kTestEpsilon);
    }
}
