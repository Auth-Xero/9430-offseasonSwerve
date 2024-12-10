// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Elastic.ElasticNotification;

/** Where all SmartDashbard vales will be edited */
public class SmartDashboardUtils extends SubsystemBase{

    private PathPlannerPath spinPath;
    private PathPlannerPath driveForwarPath;

    public SendableChooser<PathPlannerPath> pathChooser;

    private double startTime;
    private double upTime;

    /**
     * Constructor for SmartDashboardUtils
     * 
     * @param robotContainer
     */
    public SmartDashboardUtils() {
        dashboardInit();
    }

    /**
     * Initializes the values for the SmartDashboard and gets all required instances
     */
    public void dashboardInit() {
        try {

            pathChooser = new SendableChooser<PathPlannerPath>();

            spinPath = PathPlannerPath.fromPathFile("Spin");
            driveForwarPath = PathPlannerPath.fromPathFile("Drive Forward");

            pathChooser.setDefaultOption("Go Forward", driveForwarPath);
            pathChooser.addOption("Spin", spinPath);

            SmartDashboard.putData("Path Chooser", pathChooser);


            // Uptime
            startTime = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("Uptime (s)", 0);
            
            // LiveWindow
            LiveWindow.setEnabled(true);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Updates the values on the SmartDashboard
     */
    @Override
    public void periodic() {
        try {

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
