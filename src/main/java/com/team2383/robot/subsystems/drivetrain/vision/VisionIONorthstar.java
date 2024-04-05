package com.team2383.robot.subsystems.drivetrain.vision;

import com.team2383.lib.CameraParameters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;

public class VisionIONorthstar implements VisionIO {

    private final DoubleArraySubscriber observationSubscriber;
    private final IntegerSubscriber fpsSubscriber;

    private final StringPublisher tagLayoutPublisher;

    public VisionIONorthstar(String identifier, CameraParameters parameters) {
        var northstarTable = NetworkTableInstance.getDefault().getTable(identifier);

        var configTable = northstarTable.getSubTable("config");
        configTable.getIntegerTopic("camera_id").publish().set(0);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(parameters.cameraResolutionWidth);
        configTable.getIntegerTopic("camera_resolution_height").publish().set(parameters.cameraResolutionHeight);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(parameters.cameraAutoExposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(parameters.cameraExposure);
        configTable.getIntegerTopic("camera_gain").publish().set(parameters.cameraGain);
        configTable.getDoubleTopic("fiducial_size_m").publish().set(Units.inchesToMeters(6));
        tagLayoutPublisher = configTable
                .getStringTopic("tag_layout")
                .publish(PubSubOption.sendAll(true));

        var outputTable = northstarTable.getSubTable("output");
        observationSubscriber = outputTable
                .getDoubleArrayTopic("observations")
                .subscribe(
                        new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);
    }

    public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
        var queue = observationSubscriber.readQueue();
        inputs.timestamps = new double[queue.length];
        inputs.frames = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
            inputs.timestamps[i] = queue[i].serverTime / 1000000.0;
            inputs.frames[i] = queue[i].value;
        }
        inputs.fps = fpsSubscriber.get();
    }

    public void setTagPoses(Pose3d[] poses) {
        tagLayoutPublisher.set("{\r\n" + //
                "  \"tags\": [\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 1,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 2,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 3,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": {\r\n" + //
                "          \"x\": 16.579342,\r\n" + //
                "          \"y\": 4.982717999999999,\r\n" + //
                "          \"z\": 1.4511020000000001\r\n" + //
                "        },\r\n" + //
                "        \"rotation\": {\r\n" + //
                "          \"quaternion\": {\r\n" + //
                "            \"W\": 6.123233995736766e-17,\r\n" + //
                "            \"X\": 0.0,\r\n" + //
                "            \"Y\": 0.0,\r\n" + //
                "            \"Z\": 1.0\r\n" + //
                "          }\r\n" + //
                "        }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 4,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": {\r\n" + //
                "          \"x\": 16.579342,\r\n" + //
                "          \"y\": 5.547867999999999,\r\n" + //
                "          \"z\": 1.4511020000000001\r\n" + //
                "        },\r\n" + //
                "        \"rotation\": {\r\n" + //
                "          \"quaternion\": {\r\n" + //
                "            \"W\": 6.123233995736766e-17,\r\n" + //
                "            \"X\": 0.0,\r\n" + //
                "            \"Y\": 0.0,\r\n" + //
                "            \"Z\": 1.0\r\n" + //
                "          }\r\n" + //
                "        }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 5,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 6,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 7,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": {\r\n" + //
                "          \"x\": -0.038099999999999995,\r\n" + //
                "          \"y\": 5.547867999999999,\r\n" + //
                "          \"z\": 1.4511020000000001\r\n" + //
                "        },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 8,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": {\r\n" + //
                "          \"x\": -0.038099999999999995,\r\n" + //
                "          \"y\": 4.982717999999999,\r\n" + //
                "          \"z\": 1.4511020000000001\r\n" + //
                "        },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 9,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 10,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 11,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 12,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 13,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 14,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 15,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    },\r\n" + //
                "    {\r\n" + //
                "      \"ID\": 16,\r\n" + //
                "      \"pose\": {\r\n" + //
                "        \"translation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\r\n" + //
                "        \"rotation\": { \"quaternion\": { \"W\": 1.0, \"X\": 0.0, \"Y\": 0.0, \"Z\": 0.0 } }\r\n" + //
                "      }\r\n" + //
                "    }\r\n" + //
                "  ],\r\n" + //
                "  \"field\": { \"length\": 16.451, \"width\": 8.211 }\r\n" + //
                "}\r\n" + //
                "");
    }
}