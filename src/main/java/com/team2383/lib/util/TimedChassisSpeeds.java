package com.team2383.lib.util;

import java.nio.ByteBuffer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;

public class TimedChassisSpeeds {
    public final ChassisSpeeds chassisSpeeds;
    public final double timestamp;

    public static final TimedChassisSpeedsStruct struct = new TimedChassisSpeedsStruct();

    public TimedChassisSpeeds() {
        this(new ChassisSpeeds(), 0);
    }

    public TimedChassisSpeeds(ChassisSpeeds chassisSpeeds, double timestamp) {
        this.chassisSpeeds = chassisSpeeds;
        this.timestamp = timestamp;
    }

    public static class TimedChassisSpeedsStruct implements Struct<TimedChassisSpeeds> {
        @Override
        public Class<TimedChassisSpeeds> getTypeClass() {
            return TimedChassisSpeeds.class;
        }

        @Override
        public String getTypeString() {
            return "struct:ChassisSpeeds";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 4;
        }

        @Override
        public String getSchema() {
            return "double vx;double vy;double omega; double timestamp;";
        }

        @Override
        public TimedChassisSpeeds unpack(ByteBuffer bb) {
            double vx = bb.getDouble();
            double vy = bb.getDouble();
            double omega = bb.getDouble();
            double timestamp = bb.getDouble();
            return new TimedChassisSpeeds(new ChassisSpeeds(vx, vy, omega), timestamp);
        }

        @Override
        public void pack(ByteBuffer bb, TimedChassisSpeeds value) {
            bb.putDouble(value.chassisSpeeds.vxMetersPerSecond);
            bb.putDouble(value.chassisSpeeds.vyMetersPerSecond);
            bb.putDouble(value.chassisSpeeds.omegaRadiansPerSecond);
            bb.putDouble(value.timestamp);
        }
    }
}
