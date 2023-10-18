package team2383.robot;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import com.team2383.lib.controller.TunableArmFeedforward;

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
}
