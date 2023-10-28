package team2383.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import com.team2383.lib.controller.TunableArmFeedforward;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class TunableArmFeedforwardTest {
    public static final double kTestEpsilon = 1e-12; // acceptable deviation range

    @Test // marks this method as a test
    public void testConstructor() {
        TunableArmFeedforward m_instance = new TunableArmFeedforward(0.0, 0.0, 0.0, 0.0);
        Assertions.assertEquals(0.0, m_instance.ks, kTestEpsilon);
        Assertions.assertEquals(0.0, m_instance.kg, kTestEpsilon);
        Assertions.assertEquals(0.0, m_instance.kv, kTestEpsilon);
        Assertions.assertEquals(0.0, m_instance.ka, kTestEpsilon);

    }

    @Test // marks this method as a test
    public void testCalculate() {
        TunableArmFeedforward m_instance = new TunableArmFeedforward(1, 0.0, 0.0, 0.0);
        Assertions.assertEquals(1, m_instance.calculate(0.0, 1, 0.0), kTestEpsilon);
        Assertions.assertEquals(1, m_instance.calculate(0.0, 1), kTestEpsilon);
    }

    @Test
    public void poseTesting() {
        Pose3d robot = new Pose3d(0, 1, 2, new Rotation3d(3, 1.2, -1));
        Pose3d tag = new Pose3d(2, 5, 8, new Rotation3d(0.12, 0.4, 0.01));

        // calculate transform from robot to tag
        Pose3d robotToTag = tag.relativeTo(robot);

        Transform3d robotToTagTransform = new Transform3d(robotToTag.getTranslation(), robotToTag.getRotation());

        Transform3d robot_to_tag_2 = new Transform3d(robot, tag);

        System.out.println(robot);
        System.out.println(tag);

        System.out.println(robot_to_tag_2);

        assertEquals(tag, robot.plus(robotToTagTransform));
        assertEquals(tag, robot.plus(robot_to_tag_2));
        assertEquals(robotToTagTransform, robot_to_tag_2);
    }
}
