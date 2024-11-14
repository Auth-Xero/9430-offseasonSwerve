package frc.utils;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonVisionUtils {
    PhotonCamera arducam_1;
    PhotonCamera arducam_2;
    PhotonCamera arducam_3;
    PhotonCamera arducam_4;

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
            List<TargetCorner> corners = target.getDetectedCorners();

            SmartDashboard.putNumber("Target Yaw", yaw);
            SmartDashboard.putNumber("Target Pitch", pitch);
            SmartDashboard.putNumber("Target Area", area);
            SmartDashboard.putNumber("Target Skew", skew);

            SmartDashboard.putNumber("Target PoseX", pose.getX());
            SmartDashboard.putNumber("Target PoseY", pose.getY());
            SmartDashboard.putNumber("Target PoseZ", pose.getZ());

        }

        
    }
}
