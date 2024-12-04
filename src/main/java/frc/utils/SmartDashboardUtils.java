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

            SmartDashboard.putBoolean("Action Button", false);
            // Autos
            String testA = "a";
            paths.addOption("Test a", testA);
            String testB = "b";
            paths.addOption("Test b", testB);
            String testC = "c";
            paths.addOption("Test C", testC);
            SmartDashboard.putData("Paths I hope", paths);
            SmartDashboard.putString("Selected Path", selectedPath);

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

            if (SmartDashboard.getBoolean("Action Button", false)) {
                
                SmartDashboard.putBoolean("Action Button", false);

                ElasticNotification notification = new ElasticNotification();
                notification.setLevel(NotificationLevel.INFO);
                notification.setTitle("Action Button");
                notification.setDescription("It has been pressed");
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

            selectedPath = paths.getSelected();
            SmartDashboard.putString("Selected Path", selectedPath);


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