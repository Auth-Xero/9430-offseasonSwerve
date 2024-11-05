package frc.utils;

import org.photonvision.PhotonCamera;

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
}
