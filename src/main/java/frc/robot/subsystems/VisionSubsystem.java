package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;

    private final String cameraName = "GSC_BLACK";

    
    // Camera position relative to robot center (in meters)
    private final Translation2d cameraOffset;
    // x (forward), y (left/right)
    private final Rotation2d cameraYawOffset; // Camera's yaw relative to robot heading
    private final double cameraHeight; // z (height) in meters
    private final double cameraPitch; // Pitch angle in degrees

    private boolean hasTarget;
    private double poseAmbiguity;

    private PhotonTrackedTarget target;
    private int targetID;
    private Transform3d bestCameraToTargetPose;
    private static double rawYaw;
    private static double target_x;
    public static double target_y;
    private double target_z;

    private static double robotTargetX;
    private static double robotTargetY;

    
    
    public VisionSubsystem() {
        camera = new PhotonCamera(cameraName);
        latestResult = new PhotonPipelineResult();
        cameraOffset = new Translation2d(VisionConstants.cameraXOffset, VisionConstants.cameraYOffset);
        cameraYawOffset = Rotation2d.fromDegrees(VisionConstants.cameraYawOffset);
        cameraHeight = VisionConstants.cameraHeight;
        cameraPitch = VisionConstants.cameraPitch;

        target_x = 0;
        target_y = 1;
    }

    @Override
    public void periodic() {
        // query camera
        latestResult = camera.getLatestResult();
        // latestResultG = cameraG.getLatestResult();
        hasTarget = latestResult.hasTargets();
        // avoid null pointer exception if no tracked target
        if ( hasTarget ) {
            target = latestResult.getBestTarget();
            targetID = target.getFiducialId();
            poseAmbiguity = target.getPoseAmbiguity();
            bestCameraToTargetPose = target.getBestCameraToTarget();

            // cameraYawOffset is Rotation2d (radians via .getRadians())
            double yaw = cameraYawOffset.getRadians(); // camera yaw relative to robot
            double x_cam = bestCameraToTargetPose.getX(); // meters
            double y_cam = bestCameraToTargetPose.getY();

            // rotate camera->target by camera yaw, then add cameraOffset to get robot-frame target:
            double x_robot = cameraOffset.getX() + Math.cos(yaw) * x_cam - Math.sin(yaw) * y_cam;
            double y_robot = cameraOffset.getY() + Math.sin(yaw) * x_cam + Math.cos(yaw) * y_cam;

            // store robot-frame values (new getters)
            robotTargetX = x_robot;
            robotTargetY = y_robot;


            rawYaw = target.getYaw();
            target_x = bestCameraToTargetPose.getX();
            target_y = bestCameraToTargetPose.getY();
            target_z = bestCameraToTargetPose.getZ();
        }
    }


    public boolean hasTarget() {
        return hasTarget;      // mpk - should be periodic result via public gettter/setter interface?
    }

    public static double getTarget_rawYaw(){
        return rawYaw;
    }

    public static double getTarget_y() {
        return target_y;
    }

    public static double getTarget_x() {
        return target_x;
    }

    /**
     * Get yaw adjusted for camera offset
     * @param robotHeading Current robot heading (Rotation2d)
     * @return Yaw in degrees relative to robot frame
     */
    public double getTargetYawAdjusted(Rotation2d robotHeading) {
        if (hasTarget()) {
            double rawYaw = latestResult.getBestTarget().getYaw();
            // Adjust yaw for camera's orientation and position
            Rotation2d adjustedYawB = Rotation2d.fromDegrees(rawYaw).plus(cameraYawOffset).minus(robotHeading);
            return adjustedYawB.getDegrees();
        }
        return 0.0;
    }


    public double getTargetPitch() {
        if (hasTarget()) {
            return latestResult.getBestTarget().getPitch();
        }
        return 0.0;
    }

    public double getTargetArea() {
        if (hasTarget()) {
            return latestResult.getBestTarget().getArea();
        }
        return 0.0;
    }

    /**
     * Estimate distance to target, accounting for camera height and pitch
     * @param targetHeight Height of target from ground (meters)
     * @return Distance in meters, -1 if no target
     */
    public double getDistanceToTarget(double targetHeight) {
        if (hasTarget()) {
            double pitch = getTargetPitch();
            double totalPitch = Math.toRadians(cameraPitch + pitch);
            return (targetHeight - cameraHeight) / Math.tan(totalPitch);
        }
        return -1.0;
    }

    public PhotonTrackedTarget getBestTarget() {
        if (hasTarget()) {
            return target;
        }
        return null;
    }

    public Translation2d getCameraOffset() {
        return cameraOffset;
    }

    public Rotation2d getCameraYawOffset() {
        return cameraYawOffset;
    }
}