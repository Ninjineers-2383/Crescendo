package com.team2383.lib.SLAM;

import java.util.function.BiFunction;

import org.ejml.equation.Function;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class EKFSLAM {
    private final ExtendedKalmanFilterDisc ekf;
    private final SimpleMatrix F_x;

    public EKFSLAM(int numLandmarks) {
        F_x = new SimpleMatrix(7, (numLandmarks + 1) * 7);
        for (int i = 0; i < 7; i++) {
            F_x.set(i, i, 1);
        }

        SimpleMatrix init_mu = new SimpleMatrix(7 * (numLandmarks + 1), 1);
        // Set 0 rotation
        for (int i = 0; i < 7 * (numLandmarks + 1); i += 7) {
            init_mu.set(i + 3, 0, 1);
        }

        SimpleMatrix init_sigma = new SimpleMatrix(7 * (numLandmarks + 1), 7 * (numLandmarks + 1));

        for (int i = 7; i < 7 * (numLandmarks + 1); i++) {
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
        SimpleMatrix R = new SimpleMatrix(7, 7);
        ekf.predict(u, F_x.transpose().mult(R).mult(F_x));

        SimpleMatrix mu = ekf.mu();
        return new Pose3d(mu.get(0), mu.get(1), mu.get(2),
                new Rotation3d(new Quaternion(mu.get(3), mu.get(4), mu.get(5), mu.get(6))));
    }

    private class MotionModel implements BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> {
        private SimpleMatrix F_x;

        public MotionModel(SimpleMatrix F_x) {
            this.F_x = F_x;
        }

        @Override
        public SimpleMatrix apply(SimpleMatrix u, SimpleMatrix mu) {
            double dx = u.get(0);
            double dy = u.get(1);
            double dtheta = u.get(2);

            double rw = mu.get(3);
            double rx = mu.get(4);
            double ry = mu.get(5);
            double rz = mu.get(6);

            double x = mu.get(0);
            double y = mu.get(1);
            double z = mu.get(2);

            SimpleMatrix sys = new SimpleMatrix(7, 1, true,
                    rw * (dx * rw - dy * rz)
                            + rx * (dx * rx + dy * ry)
                            - ry * (dx * ry - dy * rx)
                            - rz * (dx * rz + dy * rw)
                            + x,
                    rw * (dx * rz + dy * rw)
                            + rx * (dx * ry - dy * rx)
                            + ry * (dx * rx + dy * ry)
                            + rz * (dx * rw - dy * rz)
                            + y,
                    -2 * dx * rw * ry
                            + 2 * dx * rx * rz
                            + 2 * dy * rw * rx
                            + 2 * dy * ry * rz
                            + z,
                    rw * Math.cos(dtheta / 2) - rz * Math.sin(dtheta / 2),
                    rx * Math.cos(dtheta / 2) - ry * Math.sin(dtheta / 2),
                    rx * Math.sin(dtheta / 2) + ry * Math.cos(dtheta / 2),
                    rw * Math.sin(dtheta / 2) + rz * Math.cos(dtheta / 2));

            return F_x.transpose().mult(sys);
        }
    }

    private class MotionModelJacobian implements BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> {
        private SimpleMatrix F_x;

        public MotionModelJacobian(SimpleMatrix F_x) {
            this.F_x = F_x;
        }

        @Override
        public SimpleMatrix apply(SimpleMatrix u, SimpleMatrix mu) {
            double dx = u.get(0);
            double dy = u.get(1);
            double dtheta = u.get(2);

            double rw = mu.get(3);
            double rx = mu.get(4);
            double ry = mu.get(5);
            double rz = mu.get(6);

            double[][] mat = {
                    {
                            1,
                            0,
                            0,
                            2 * dx * rw - 2 * dy * rz,
                            2 * dx * rx + 2 * dy * ry,
                            -2 * dx * ry + 2 * dy * rx,
                            -2 * dx * rz - 2 * dy * rw,
                    },
                    {
                            0,
                            1,
                            0,
                            2 * dx * rz + 2 * dy * rw,
                            2 * dx * ry - 2 * dy * rx,
                            2 * dx * rx + 2 * dy * ry,
                            2 * dx * rw - 2 * dy * rz,
                    },
                    {
                            0,
                            0,
                            1,
                            -2 * dx * ry + 2 * dy * rx,
                            2 * dx * rz + 2 * dy * rw,
                            -2 * dx * rw + 2 * dy * rz,
                            2 * dx * rx + 2 * dy * ry,
                    },
                    {
                            0,
                            0,
                            0,
                            Math.cos(dtheta / 2),
                            0,
                            0,
                            -Math.sin(dtheta / 2)
                    },
                    {
                            0,
                            0,
                            0,
                            0,
                            Math.cos(dtheta / 2),
                            -Math.sin(dtheta / 2), 0
                    },
                    {
                            0,
                            0,
                            0,
                            0,
                            Math.sin(dtheta / 2),
                            Math.cos(dtheta / 2),
                            0
                    },
                    {
                            0,
                            0,
                            0,
                            Math.sin(dtheta / 2),
                            0,
                            0,
                            Math.cos(dtheta / 2)
                    }
            };

            SimpleMatrix G = new SimpleMatrix(mat);

            return F_x.transpose().mult(G).mult(F_x);
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
