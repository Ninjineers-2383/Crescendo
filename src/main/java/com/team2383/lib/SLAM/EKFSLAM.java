package com.team2383.lib.SLAM;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;

public class EKFSLAM {
    private final SimpleMatrix F_x;

    private SimpleMatrix mu;
    private SimpleMatrix sigma;

    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> motion_model;
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> motion_model_jacobian;
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> sensor_model;
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> sensor_model_jacobian;

    boolean[] seenLandmarks;

    public EKFSLAM(int numLandmarks) {
        F_x = new SimpleMatrix(7, (numLandmarks + 1) * 7);
        for (int i = 0; i < 7; i++) {
            F_x.set(i, i, 1);
        }

        mu = new SimpleMatrix(7 * (numLandmarks + 1), 1);
        // Set 0 rotation
        for (int i = 0; i < 7 * (numLandmarks + 1); i += 7) {
            mu.set(i + 3, 0, 1);
        }

        sigma = new SimpleMatrix(7 * (numLandmarks + 1), 7 * (numLandmarks + 1));

        for (int i = 0; i < 7; i++) {
            // for (int j = 0; j < 63; j++) {
            // sigma.set(i, j, 0);
            // sigma.set(j, i, 0);
            // }
            sigma.set(i, i, 0);
        }

        for (int i = 7; i < 7 * (numLandmarks + 1); i++) {
            sigma.set(i, i, 0.5);
        }

        motion_model = new MotionModel(F_x);
        motion_model_jacobian = new MotionModelJacobian(F_x);
        sensor_model = new SensorModelTag();
        sensor_model_jacobian = new SensorModelTagJacobian();

        seenLandmarks = new boolean[numLandmarks];
    }

    public void seedLandmarks(Pose3d[] landmarks) {
        for (int i = 0; i < landmarks.length; i++) {
            if (landmarks[i] != null) {
                mu.set(7 * (i + 1), 0, landmarks[i].getTranslation().getX());
                mu.set(7 * (i + 1) + 1, 0, landmarks[i].getTranslation().getY());
                mu.set(7 * (i + 1) + 2, 0, landmarks[i].getTranslation().getZ());
                mu.set(7 * (i + 1) + 3, 0, landmarks[i].getRotation().getQuaternion().getW());
                mu.set(7 * (i + 1) + 4, 0, landmarks[i].getRotation().getQuaternion().getX());
                mu.set(7 * (i + 1) + 5, 0, landmarks[i].getRotation().getQuaternion().getY());
                mu.set(7 * (i + 1) + 6, 0, landmarks[i].getRotation().getQuaternion().getZ());
                seenLandmarks[i] = true;
            }
        }

        Pose3d[] landmarks2 = new Pose3d[seenLandmarks.length];

        for (int i = 0; i < seenLandmarks.length; i++) {
            if (seenLandmarks[i]) {
                Pose3d landmark = getPose(mu, 7 * (i + 1));
                landmarks2[i] = landmark;
            } else {
                landmarks2[i] = new Pose3d();
            }
        }

        Logger.recordOutput("SLAM/landmarks", landmarks2);
    }

    public Pose3d predict(ChassisSpeeds speeds, double dt) {
        SimpleMatrix u = new SimpleMatrix(3, 1, true,
                speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt, speeds.omegaRadiansPerSecond * dt);
        SimpleMatrix R = SimpleMatrix.diag(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

        mu = mu.plus(motion_model.apply(u, mu));
        SimpleMatrix G = SimpleMatrix.identity(63).plus(motion_model_jacobian.apply(u, mu));
        sigma = G.mult(sigma.mult(G.transpose())).plus(F_x.transpose().mult(R.mult(F_x)));

        return getPose(mu, 0);
    }

    public Pose3d correct(Transform3d robotToTag, int landmarkIndex) {
        SimpleMatrix z_obs = new SimpleMatrix(7, 1, true,
                robotToTag.getTranslation().getX(), robotToTag.getTranslation().getY(),
                robotToTag.getTranslation().getZ(),
                robotToTag.getRotation().getQuaternion().getW(),
                robotToTag.getRotation().getQuaternion().getX(),
                robotToTag.getRotation().getQuaternion().getY(),
                robotToTag.getRotation().getQuaternion().getZ());

        if (!seenLandmarks[landmarkIndex]) {
            seenLandmarks[landmarkIndex] = true;
            Pose3d tag = getRobotPose().plus(robotToTag);
            mu.set(7 * (landmarkIndex + 1), 0, tag.getTranslation().getX());
            mu.set(7 * (landmarkIndex + 1) + 1, 0, tag.getTranslation().getY());
            mu.set(7 * (landmarkIndex + 1) + 2, 0, tag.getTranslation().getZ());
            mu.set(7 * (landmarkIndex + 1) + 3, 0, tag.getRotation().getQuaternion().getW());
            mu.set(7 * (landmarkIndex + 1) + 4, 0, tag.getRotation().getQuaternion().getX());
            mu.set(7 * (landmarkIndex + 1) + 5, 0, tag.getRotation().getQuaternion().getY());
            mu.set(7 * (landmarkIndex + 1) + 6, 0, tag.getRotation().getQuaternion().getZ());
        }

        SimpleMatrix z_pred = sensor_model.apply(mu, new SimpleMatrix(1, 1, true,
                landmarkIndex));

        SimpleMatrix H = sensor_model_jacobian.apply(mu, new SimpleMatrix(1, 1, true,
                landmarkIndex));

        SimpleMatrix F_xj = new SimpleMatrix(14, mu.getNumRows());
        for (int i = 0; i < 7; i++) {
            F_xj.set(i, i, 1);
            F_xj.set(i + 7, i + 7 * (landmarkIndex + 1), 1);
        }

        SimpleMatrix Hi = H.mult(F_xj);

        SimpleMatrix Q = SimpleMatrix.diag(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

        SimpleMatrix S = Hi.mult(sigma.mult(Hi.transpose())).plus(Q).invert();

        SimpleMatrix K = sigma.mult(Hi.transpose()).mult(S);

        mu = mu.plus(K.mult(subtractPose(z_obs, z_pred)));
        sigma = (SimpleMatrix.identity(K.getNumRows()).minus(K.mult(Hi))).mult(sigma);

        // Logger.recordOutput("SLAM/sigma", sigma.getDDRM().data);

        NetworkTableInstance.getDefault().getTable("SLAM").getEntry("sigma").setDoubleArray(sigma.getDDRM().data);

        Pose3d robotPose = getPose(mu, 0);

        Pose3d[] landmarks = new Pose3d[seenLandmarks.length];

        for (int i = 0; i < seenLandmarks.length; i++) {
            if (seenLandmarks[i]) {
                Pose3d landmark = getPose(mu, 7 * (i + 1));
                landmarks[i] = landmark;
            } else {
                landmarks[i] = new Pose3d();
            }
        }

        Logger.recordOutput("SLAM/landmarks", landmarks);

        return robotPose;
    }

    public Pose3d getRobotPose() {
        return getPose(mu, 0);
    }

    private SimpleMatrix subtractPose(SimpleMatrix A, SimpleMatrix B) {
        if (Math.signum(A.get(6)) != Math.signum(B.get(6))) {
            A.set(3, 0, -A.get(3));
            A.set(4, 0, -A.get(4));
            A.set(5, 0, -A.get(5));
            A.set(6, 0, -A.get(6));
        }

        return A.minus(B);
    }

    private Pose3d getPose(SimpleMatrix mu, int start) {
        return new Pose3d(mu.get(0 + start), mu.get(1 + start), mu.get(2 + start),
                new Rotation3d(
                        new Quaternion(mu.get(3 + start), mu.get(4 + start), mu.get(5 + start), mu.get(6 + start))));
    }
}
