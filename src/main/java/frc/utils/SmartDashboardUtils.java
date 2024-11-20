// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

/** Where all SmartDashbard vales will be edited */
public class SmartDashboardUtils {

    private RobotContainer robotContainer;
    private DriveSubsystem driveSubsystem;
    private XboxController driverController;
    private Pigeon2 pigeon;

    private double startTime;
    private double upTime;

    private int actionsRan;

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
            SmartDashboard.putBoolean("Driver Controller A button", false);
            SmartDashboard.putBoolean("Driver Controller B button", false);

            // Pigeon2 Gyro
            SmartDashboard.putNumber("Pigeon2 Rate", 0);
            SmartDashboard.putNumber("Pigeon2 Angle", 0);

            // Input Testing
            SmartDashboard.putNumber("Input Test Num", 0);
            SmartDashboard.putNumber("Output Test Num", 0);
            SmartDashboard.putBoolean("Input Test Bool", false);
            SmartDashboard.putBoolean("Output Test Bool", false);
            SmartDashboard.putString("Input Test String", "");
            SmartDashboard.putString("Output Test String", "");
            SmartDashboard.putBoolean("Action Button", false);
            actionsRan = 0;
            SmartDashboard.putNumber("Commands Run", actionsRan);

            // Uptime
            startTime = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("Uptime", 0);

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
            SmartDashboard.putBoolean("Driver Controller A button", driverController.getAButton());
            SmartDashboard.putBoolean("Driver Controller B button", driverController.getBButton());

            // Pigeon2 Gyro
            SmartDashboard.putNumber("Pigeon2 Rate", pigeon.getRate());
            SmartDashboard.putNumber("Pigeon2 Angle", pigeon.getAngle());

            // Input Testing
            SmartDashboard.putNumber("Output Test Num", SmartDashboard.getNumber("Input Test Num", 0));
            SmartDashboard.putBoolean("Output Test Bool", SmartDashboard.getBoolean("Input Test Bool", false));
            SmartDashboard.putString("Output Test String", SmartDashboard.getString("Input Test String", ""));

            if(SmartDashboard.getBoolean("Action Button", false)) {
                actionsRan ++;
                SmartDashboard.putBoolean("Action Button", false);
                SmartDashboard.putNumber("Commands Run", actionsRan);
            }

            //Uptime
            upTime = Timer.getFPGATimestamp() - startTime;
            SmartDashboard.putNumber("Uptime (s)", (int)upTime);

            // LiveWindow
            LiveWindow.updateValues();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
