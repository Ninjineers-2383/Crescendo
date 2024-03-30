package com.team2383.lib;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;

public class CameraParameters {
    public final int cameraResolutionWidth;
    public final int cameraResolutionHeight;
    public final int cameraAutoExposure;
    public final int cameraExposure;
    public final int cameraGain;

    public CameraParameters(int cameraResolutionWidth, int cameraResolutionHeight, int cameraAutoExpose,
            int cameraExpose, int cameraGain) {
        this.cameraResolutionWidth = cameraResolutionWidth;
        this.cameraResolutionHeight = cameraResolutionHeight;
        this.cameraAutoExposure = cameraAutoExpose;
        this.cameraExposure = cameraExpose;
        this.cameraGain = cameraGain;
    }

    public CameraParameters() {
        this(1280, 720, 1, 1000, 0);
    }

    public static CameraParametersStruct struct = new CameraParametersStruct();

    public static class CameraParametersStruct implements Struct<CameraParameters> {
        @Override
        public Class<CameraParameters> getTypeClass() {
            return CameraParameters.class;
        }

        @Override
        public String getTypeString() {
            return "struct:CameraParameters";
        }

        @Override
        public int getSize() {
            return kSizeInt32 * 5;
        }

        @Override
        public String getSchema() {
            return "int width;int height;int exposeAuto;int expose;int gain";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {};
        }

        @Override
        public CameraParameters unpack(ByteBuffer bb) {
            return new CameraParameters(
                    bb.getInt(),
                    bb.getInt(),
                    bb.getInt(),
                    bb.getInt(),
                    bb.getInt());
        }

        @Override
        public void pack(ByteBuffer bb, CameraParameters value) {
            bb.putInt(value.cameraResolutionWidth);
            bb.putInt(value.cameraResolutionHeight);
            bb.putInt(value.cameraAutoExposure);
            bb.putInt(value.cameraExposure);
            bb.putInt(value.cameraGain);
        }
    }
}
