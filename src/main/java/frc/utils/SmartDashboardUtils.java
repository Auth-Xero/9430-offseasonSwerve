// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.nio.file.Path;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.Elastic.ElasticNotification;

/** Where all SmartDashbard vales will be edited */
public class SmartDashboardUtils extends SubsystemBase{

    private DriveSubsystem driveSubsystem;
    private Pigeon2 gyro;

    private PathPlannerPath spinPath;
    private PathPlannerPath driveForwarPath;

    public SendableChooser<PathPlannerPath> pathChooser;

    /**
     * Constructor for SmartDashboardUtils
     * 
     * @param robotContainer
     */
    public SmartDashboardUtils(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.gyro = driveSubsystem.getPigeon2();
        dashboardInit();
    }

    /**
     * Initializes the values for the SmartDashboard and gets all required instances
     */
    public void dashboardInit() {
        try {

            pathChooser = new SendableChooser<PathPlannerPath>();

            spinPath = PathPlannerPath.fromPathFile("Spin");
            driveForwarPath = PathPlannerPath.fromPathFile("Drive Forward " + ((DriverStation.getAlliance().get() == Alliance.Red)? "Red": "Blue"));

            pathChooser.setDefaultOption("Go Forward " + ((DriverStation.getAlliance().get() == Alliance.Red)? "Red": "Blue"), driveForwarPath);
            pathChooser.addOption("Spin", spinPath);

            SmartDashboard.putData("Path Chooser", pathChooser);

            SmartDashboard.putBoolean("Zero Heading", false);


           

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
            
            if(SmartDashboard.getBoolean("Zero Heading", false)) {
                SmartDashboard.putBoolean("Zero Heading", false);
                driveSubsystem.zeroHeading();
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void sendElasticNotification(ElasticNotification notification){
        Elastic.sendAlert(notification);
    }

}
