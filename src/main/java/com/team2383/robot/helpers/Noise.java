package com.team2383.robot.helpers;

import java.util.Random;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Noise {
    private Noise() {
    }

    public static Transform3d noisyTransform(double mu, double sigma) {
        Random r = new Random();

        double x = r.nextGaussian() * sigma + mu;
        double y = r.nextGaussian() * sigma + mu;
        double z = r.nextGaussian() * sigma + mu;

        double roll = r.nextGaussian() * sigma + mu;
        double pitch = r.nextGaussian() * sigma + mu;
        double yaw = r.nextGaussian() * sigma + mu;

        return new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
    }

    public static ChassisSpeeds noisySpeeds(double mu, double sigma) {
        Random r = new Random();

        double vx = r.nextGaussian() * sigma + mu;
        double vy = r.nextGaussian() * sigma + mu;
        double omega = r.nextGaussian() * sigma + mu;

        return new ChassisSpeeds(vx, vy, omega);
    }
}
