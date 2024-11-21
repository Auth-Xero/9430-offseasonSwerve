package frc.utils;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonVisionUtils {
    PhotonCamera arducam_1;
    PhotonCamera arducam_2;
    PhotonCamera arducam_3;
    PhotonCamera arducam_4;
    
    private NetworkTable visionTable;

    // LED Mode Enum
    public enum LEDMode {
        Off(0),
        On(1);

        private final int value;

        LEDMode(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    

    public PhotonVisionUtils() {

        arducam_1 = new PhotonCamera("Arducam_1");
        arducam_2 = new PhotonCamera("Arducam_2");
        arducam_3 = new PhotonCamera("Arducam_3");
        arducam_4 = new PhotonCamera("Arducam_4");

    }

    public void readCameraData(PhotonCamera camera) {
        
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        PhotonPipelineResult result = results.get(0);

        boolean hasTargets = result.hasTargets();

        if(hasTargets) {
            PhotonTrackedTarget target = result.getBestTarget();

            // Get information from target.
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();
            Transform3d pose = target.getBestCameraToTarget();

            SmartDashboard.putNumber("Target Yaw", yaw);
            SmartDashboard.putNumber("Target Pitch", pitch);
            SmartDashboard.putNumber("Target Area", area);
            SmartDashboard.putNumber("Target Skew", skew);

            SmartDashboard.putNumber("Target PoseX", pose.getX());
            SmartDashboard.putNumber("Target PoseY", pose.getY());
            SmartDashboard.putNumber("Target PoseZ", pose.getZ());

        }

        
    }

    public void InnerPhotonVisionUtils(String cameraName) {
        // Getting the default instance of NetworkTables
        var inst = NetworkTableInstance.getDefault();
        // Getting the table for PhotonVision
        visionTable = inst.getTable("photonvision/" + cameraName);
    }

    public void periodic() {
        // Periodic updates can be handled here if necessary
    }

    public boolean hasTarget() {
        return visionTable.getEntry("hasTargets").getDouble(0.0) == 1.0;
    }

    public double getTargetX() {
        return visionTable.getEntry("targetYaw").getDouble(0.0);
    }

    public double getTargetY() {
        return visionTable.getEntry("targetPitch").getDouble(0.0);
    }

    public double getTargetArea() {
        return visionTable.getEntry("targetArea").getDouble(0.0);
    }

    public void setLEDMode(LEDMode mode) {
        visionTable.getEntry("ledMode").setNumber(mode.getValue());
    }

    public void setLEDOn() {
        setLEDMode(LEDMode.On);
    }

    public void setLEDOff() {
        setLEDMode(LEDMode.Off);
    }

    public double calculateDistanceToTarget(boolean isRed) {
        double photonVisionMountAngleDegrees = 30.0;
        double photonVisionLensHeightInches = 5.0;
        double goalHeightInches = isRed ? 57.5 : 54.0; // Adjust if necessary

        double angleToGoalDegrees = photonVisionMountAngleDegrees + getTargetY();
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        // Calculate distance
        return (goalHeightInches - photonVisionLensHeightInches) / Math.tan(angleToGoalRadians);
    }
}