package com.team2383.lib.util;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.struct.Struct;

public class TimedRobotUpdate {
    public final SwerveModulePosition[] modulePositions;
    public final Rotation3d gyroAngle;
    public final double timestamp;

    public static final TimeRobotUpdateStruct struct = new TimeRobotUpdateStruct();

    public TimedRobotUpdate(SwerveModulePosition[] modulePositions, Rotation3d gyroAngle, double timestamp) {
        if (modulePositions.length != 4) {
            throw new IllegalArgumentException("modulePositions must be length 4");
        }
        this.modulePositions = modulePositions;
        this.gyroAngle = gyroAngle;
        this.timestamp = timestamp;
    }

    public TimedRobotUpdate() {
        this.modulePositions = new SwerveModulePosition[4];
        this.gyroAngle = new Rotation3d();
        this.timestamp = 0;
    }

    public static class TimeRobotUpdateStruct implements Struct<TimedRobotUpdate> {
        @Override
        public Class<TimedRobotUpdate> getTypeClass() {
            return TimedRobotUpdate.class;
        }

        @Override
        public String getTypeString() {
            return "struct:TimedRobotUpdate";
        }

        @Override
        public int getSize() {
            return SwerveModulePosition.struct.getSize() * 4 + Rotation3d.struct.getSize() + kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "SwerveModulePosition[4] modulePositions;Rotation3d gyroAngle;double timestamp;";
        }

        @Override
        public TimedRobotUpdate unpack(ByteBuffer bb) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++) {
                modulePositions[i] = SwerveModulePosition.struct.unpack(bb);
            }
            Rotation3d gyroAngle = Rotation3d.struct.unpack(bb);
            double timestamp = bb.getDouble();
            return new TimedRobotUpdate(modulePositions, gyroAngle, timestamp);
        }

        @Override
        public void pack(ByteBuffer bb, TimedRobotUpdate value) {
            for (int i = 0; i < 4; i++) {
                SwerveModulePosition.struct.pack(bb, value.modulePositions[i]);
            }
            Rotation3d.struct.pack(bb, value.gyroAngle);
            bb.putDouble(value.timestamp);
        }
    }
}
