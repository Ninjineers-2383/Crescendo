package com.team2383.lib.util;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;

public class TimedPose3d {
    public final Pose3d pose;
    public final double timestamp;

    public TimedPose3d(Pose3d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }

    public TimedPose3d() {
        this.pose = new Pose3d();
        this.timestamp = 0;
    }

    public static TimedPose3dStruct struct = new TimedPose3dStruct();

    public static class TimedPose3dStruct implements Struct<TimedPose3d> {
        @Override
        public Class<TimedPose3d> getTypeClass() {
            return TimedPose3d.class;
        }

        @Override
        public String getTypeString() {
            return "struct:TimedPose3d";
        }

        @Override
        public int getSize() {
            return Pose3d.struct.getSize() + kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "Pose3d pose;double timestamp;";
        }

        @Override
        public TimedPose3d unpack(ByteBuffer bb) {
            Pose3d pose = Pose3d.struct.unpack(bb);
            double timestamp = bb.getDouble();
            return new TimedPose3d(pose, timestamp);
        }

        @Override
        public void pack(ByteBuffer bb, TimedPose3d value) {
            Pose3d.struct.pack(bb, value.pose);
            bb.putDouble(value.timestamp);
        }
    }
}
