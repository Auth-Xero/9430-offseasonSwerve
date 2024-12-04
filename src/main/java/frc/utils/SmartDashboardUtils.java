// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.Elastic.ElasticNotification;
import frc.utils.Elastic.ElasticNotification.NotificationLevel;

/** Where all SmartDashbard vales will be edited */
public class SmartDashboardUtils {

    private RobotContainer robotContainer;
    private DriveSubsystem driveSubsystem;
    private XboxController driverController;
    private Pigeon2 pigeon;

    private double startTime;
    private double upTime;


    private SendableChooser<String> paths = new SendableChooser<String>();
    String selectedPath = "";

    /**
     * Constructor for SmartDashboardUtils
     * 
     * @param robotContainer
     */
    public SmartDashboardUtils(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    /**
     * Initializes the values for the SmartDashboard and gets all required instances
     */
    public void dashboardInit() {
        try {
            driveSubsystem = robotContainer.getDriveSubsystem();
            pigeon = driveSubsystem.getPigeon2();
            driverController = robotContainer.getDriverController();

            // Driver Controller Values

            // Pigeon2 Gyro
            SmartDashboard.putBoolean("Reset Pigeon", false);

            // Uptime
            startTime = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("Uptime (s)", 0);

            // Autos
            String driveForwardPath = "pathplanner\\paths\\Drive Forward.path";
            paths.addOption("DriveForwardPath", driveForwardPath);
            String spinPath = "pathplanner\\paths\\Spin.path";
            paths.addOption("Spin", spinPath);

            paths.setDefaultOption("DriveForwardPath", driveForwardPath);

            SmartDashboard.putData("Paths", paths);

            SmartDashboard.putString("Selected Path", selectedPath);

            SmartDashboard.putBoolean("Run Auto", false);

            // LiveWindow
            LiveWindow.setEnabled(true);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Updates the values on the SmartDashboard
     */
    public void updateDashboard() {
        try {
            // Driver Controller Values
            
            // Pigeon2 Gyro

            selectedPath = paths.getSelected();
            SmartDashboard.putString("Selected Path", selectedPath);

            if (SmartDashboard.getBoolean("Run Auto", false)) {

                SmartDashboard.putBoolean("Run Auto", false);

                robotContainer.setActivePath(selectedPath);

                ElasticNotification notification = new ElasticNotification();
                notification.setLevel(NotificationLevel.INFO);
                notification.setTitle("Running Auto...");
                notification.setDescription("Selected Path: " + selectedPath);
                notification.setDisplayTimeSeconds(2);
                sendElasticNotification(notification);
            }

            if (SmartDashboard.getBoolean("Reset Pigeon", false)) {
                SmartDashboard.putBoolean("Reset Pigeon", false);
                driveSubsystem.zeroHeading();

                ElasticNotification notification = new ElasticNotification();
                notification.setLevel(NotificationLevel.INFO);
                notification.setTitle("Reset Pigeon");
                notification.setDescription("Pigeon2 has been reset");
                notification.setDisplayTimeSeconds(5);
                sendElasticNotification(notification);
            }

            // Uptime
            upTime = Timer.getFPGATimestamp() - startTime;
            SmartDashboard.putNumber("Uptime (s)", (int) upTime);

            // LiveWindow
            LiveWindow.updateValues();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void sendElasticNotification(ElasticNotification notification){
        Elastic.sendAlert(notification);
    }

}
