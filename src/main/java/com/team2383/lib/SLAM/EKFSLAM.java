package com.team2383.lib.SLAM;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class EKFSLAM {
    private final ExtendedKalmanFilterDisc ekf;
    private final SimpleMatrix F_x;

    public EKFSLAM(int numLandmarks) {
        F_x = new SimpleMatrix(6, (numLandmarks + 1) * 6);
        F_x.set(0, 0, 1);
        F_x.set(1, 1, 1);
        F_x.set(5, 5, 1);

        SimpleMatrix init_mu = new SimpleMatrix(6 * (numLandmarks + 1), 1);
        SimpleMatrix init_sigma = new SimpleMatrix(6 * (numLandmarks + 1), 6 * (numLandmarks + 1));

        for (int i = 6; i < 6 * (numLandmarks + 1); i++) {
            init_sigma.set(i, i, Double.POSITIVE_INFINITY);
        }

        ekf = new ExtendedKalmanFilterDisc(
                init_mu, init_sigma,
                new MotionModel(F_x), new MotionModelJacobian(F_x),
                null, null);
    }

    public Pose3d update(ChassisSpeeds speeds, double dt) {
        SimpleMatrix u = new SimpleMatrix(3, 1, true,
                speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt, speeds.omegaRadiansPerSecond * dt);
        SimpleMatrix R = new SimpleMatrix(6, 6);
        ekf.predict(u, F_x.transpose().mult(R).mult(F_x));

        SimpleMatrix mu = ekf.mu();
        return new Pose3d(mu.get(0), mu.get(1), mu.get(2), new Rotation3d(mu.get(3), mu.get(4), mu.get(5)));
    }

    private class MotionModel implements BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> {
        private SimpleMatrix F_x;

        public MotionModel(SimpleMatrix F_x) {
            this.F_x = F_x;
        }

        @Override
        public SimpleMatrix apply(SimpleMatrix u, SimpleMatrix mu) {
            SimpleMatrix sys = new SimpleMatrix(6, 1, true,
                    u.get(0, 0) * Math.cos(mu.get(5, 0)) - u.get(1, 0) * Math.sin(mu.get(5, 0)),
                    u.get(0, 0) * Math.sin(mu.get(5, 0)) + u.get(1, 0) * Math.cos(mu.get(5, 0)),
                    0,
                    0,
                    0,
                    u.get(2, 0));

            return mu.plus(F_x.transpose().mult(sys));
        }
    }

    private class MotionModelJacobian implements BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> {
        private SimpleMatrix F_x;

        public MotionModelJacobian(SimpleMatrix F_x) {
            this.F_x = F_x;
        }

        @Override
        public SimpleMatrix apply(SimpleMatrix u, SimpleMatrix mu) {
            SimpleMatrix G = new SimpleMatrix(6, 6);

            G.set(0, 5, -u.get(0, 0) * Math.sin(mu.get(5, 0)) - u.get(1, 0) * Math.cos(mu.get(5, 0)));
            G.set(1, 5, u.get(0, 0) * Math.cos(mu.get(5, 0)) - u.get(1, 0) * Math.sin(mu.get(5, 0)));

            return SimpleMatrix.identity(F_x.getNumCols()).plus(F_x.transpose().mult(G).mult(F_x));
        }
    }

    // private class SensorModel implements BiFunction<SimpleMatrix, SimpleMatrix,
    // SimpleMatrix> {
    // @Override
    // public SimpleMatrix apply(SimpleMatrix mu, SimpleMatrix u) {
    // throw new UnsupportedOperationException("Unimplemented method 'apply'");
    // }
    // }

}
