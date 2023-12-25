package com.team2383.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;

public class VisionIONorthstar implements VisionIO {
    private static final int cameraId = 0;
    private static final int cameraResolutionWidth = 1280;
    private static final int cameraResolutionHeight = 720;
    private static final int cameraAutoExposure = 1;
    private static final int cameraExposure = 10;
    private static final int cameraGain = 25;

    private final DoubleArraySubscriber observationSubscriber;
    private final IntegerSubscriber fpsSubscriber;

    private final StringPublisher tagLayoutPublisher;

    public VisionIONorthstar(String identifier) {
        var northstarTable = NetworkTableInstance.getDefault().getTable(identifier);

        var configTable = northstarTable.getSubTable("config");
        configTable.getIntegerTopic("camera_id").publish().set(cameraId);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
        configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
        configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
        configTable.getDoubleTopic("fiducial_size_m").publish().set(Units.inchesToMeters(6));
        tagLayoutPublisher = configTable
                .getStringTopic("tag_layout")
                .publish();

        tagLayoutPublisher.set("");

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
            inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
            inputs.frames[i] = queue[i].value;
        }
        inputs.fps = fpsSubscriber.get();
    }
}