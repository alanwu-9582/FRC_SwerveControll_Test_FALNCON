package frc.robot.subsystems;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.Constants.FieldConstants;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class VisionManager extends SubsystemBase {
    private static final VisionManager instance = new VisionManager();

    public static VisionManager getInstance() {
        return instance;
    }

    DoubleSubscriber tidSub = NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic("tid").subscribe(-1);
    DoubleArraySubscriber targetpose_robotspaceSub = NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);

    public VisionManager() {

    }

    public Transform3d getAprilTagRelative() {
        var bestCameraToTargetArray = targetpose_robotspaceSub.get();
        Transform3d bestCameraToTarget = new Transform3d();
        if (hasTarget()) {
            bestCameraToTarget = new Transform3d(
                    new Translation3d (bestCameraToTargetArray[0], -bestCameraToTargetArray[1], bestCameraToTargetArray[2]),
                    new Rotation3d(bestCameraToTargetArray[3], bestCameraToTargetArray[4], bestCameraToTargetArray[5]));
        }
        return bestCameraToTarget;
    }

    public boolean hasTarget() {
        return tidSub.get() != -1.;
    }

    boolean isFirstConnected = true;
    @Override
    public void periodic() {
    }
}