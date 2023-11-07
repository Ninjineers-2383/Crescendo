package com.team2383.lib.SLAM;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * EKFSLAM is a Simultaneous Localization and Mapping algorithm that uses an
 * extended Kalman filter to estimate the
 * robot's pose and the pose of landmarks in the environment.
 * <p>
 * The state vector is a 7x1 matrix of the form [x, y, z, qw, qx, qy, qz] where
 * x, y, and z are the robot's position in
 * meters and qw, qx, qy, and qz are the components of the robot's quaternion
 * orientation.
 * The state vector is augmented with the pose of each landmark in the
 * environment.
 */
public class EKFSLAM {
    // 7 x n matrix that maps the robot state to the full state vector.
    private final SimpleMatrix F_x;

    // Mean of the state vector.
    private SimpleMatrix mu;
    // Covariance of the state vector.
    private SimpleMatrix sigma;

    // Function that takes the robot's control input and the
    // current state vector and returns the predicted change in the state vector.
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> motion_model;
    // Function that takes the robot's control input
    // and the current state vector and returns the Jacobian of the motion model
    // w.r.t. the state vector.
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> motion_model_jacobian;
    // Function that takes the current state vector and a
    // landmark index and returns the predicted observation of the landmark.
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> sensor_model;
    // Function that takes the current state vector
    // and a landmark index and returns the Jacobian of the sensor model w.r.t. the
    // state vector of the robot and landmark.
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> sensor_model_jacobian;

    // Keeps track of which landmarks have been seen
    private final boolean[] seenLandmarks;

    /**
     * Constructs an EKFSLAM object.
     *
     * @param numLandmarks
     *            the number of landmarks in the environment
     */
    public EKFSLAM(int numLandmarks) {
        // Construct F_x matrix with 1s on the left block diagonal
        F_x = new SimpleMatrix(7, (numLandmarks + 1) * 7);
        for (int i = 0; i < 7; i++) {
            F_x.set(i, i, 1);
        }

        // Initialize empty state vector
        mu = new SimpleMatrix(7 * (numLandmarks + 1), 1);
        // Set 0 rotation quaternion
        for (int i = 0; i < 7 * (numLandmarks + 1); i += 7) {
            mu.set(i + 3, 0, 1);
        }

        // Initialize covariance matrix
        sigma = new SimpleMatrix(7 * (numLandmarks + 1), 7 * (numLandmarks + 1));
        for (int i = 7; i < 7 * (numLandmarks + 1); i++) {
            sigma.set(i, i, 0.5);
            // sigma.set(i, i, 0);
        }

        // Initialize motion model and sensor model
        motion_model = new MotionModel(F_x);
        motion_model_jacobian = new MotionModelJacobian(F_x);
        sensor_model = new SensorModelTag();
        sensor_model_jacobian = new SensorModelTagJacobian();

        seenLandmarks = new boolean[numLandmarks];
    }

    /**
     * Seeds the EKFSLAM algorithm with the initial pose of landmarks in the
     * environment. This should be called before the first call to predict() or
     * correct().
     * <p>
     * If a landmark is not seen, the corresponding pose should be null.
     * <p>
     * Useful for seeding the algorithm with the official field map at the start
     * of a match in order to get semi-accurate pose estimates before a map is
     * constructed.
     * 
     * @param landmarks
     *            an array of poses of the landmarks in the environment
     */
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
    }

    /**
     * Extended Kalman filter prediction step.
     * <p>
     * Updates the state vector and covariance matrix based on the robot's
     * control input.
     * 
     * @param speeds
     *            the robot's control input
     * @param dt
     *            the time elapsed since the last call to predict()
     * @return the predicted pose of the robot
     */
    public Pose3d predict(ChassisSpeeds speeds, double dt) {
        SimpleMatrix u = new SimpleMatrix(3, 1, true,
                speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt, speeds.omegaRadiansPerSecond * dt);
        SimpleMatrix R = SimpleMatrix.diag(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

        mu = mu.plus(motion_model.apply(u, mu));
        SimpleMatrix G = SimpleMatrix.identity(63).plus(motion_model_jacobian.apply(u, mu));
        sigma = G.mult(sigma.mult(G.transpose())).plus(F_x.transpose().mult(R.mult(F_x)));

        return getPose(mu, 0);
    }

    /**
     * Extended Kalman filter correction step.
     * <p>
     * Updates the state vector and covariance matrix based on the robot's
     * observation of a landmark.
     * 
     * @param robotToTag
     *            the robot's observation of the landmark
     * @param landmarkIndex
     *            the index of the landmark in the state vector
     * @return the corrected pose of the robot
     */
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

        Pose3d robotPose = getPose(mu, 0);
        return robotPose;
    }

    /**
     * Returns the pose of the robot.
     * 
     * @return the pose of the robot
     */
    public Pose3d getRobotPose() {
        return getPose(mu, 0);
    }

    /**
     * Returns the pose of a landmark.
     * 
     * @param landmarkIndex
     *            the index of the landmark in the state vector
     * @return the pose of the landmark
     */
    public Pose3d getLandmarkPose(int landmarkIndex) {
        return getPose(mu, 7 * (landmarkIndex + 1));
    }

    /**
     * Returns the poses of all landmarks.
     * 
     * @return the poses of all landmarks
     */
    public Pose3d[] getLandmarkPoses() {
        Pose3d[] poses = new Pose3d[(mu.getNumRows() - 7) / 7];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = getPose(mu, 7 * (i + 1));
        }
        return poses;
    }

    /**
     * Returns the covariance matrix
     * 
     * @return the covariance matrix: sigma
     */
    public SimpleMatrix getCovariance() {
        return sigma;
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
