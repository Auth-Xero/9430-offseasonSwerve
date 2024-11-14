// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class SmartDashboardUtils {

    private RobotContainer robotContainer;
    private DriveSubsystem driveSubsystem;
    private XboxController driverController;
    private Pigeon2 pigeon;

    public SmartDashboardUtils(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void dashboardInit() {
        try{
            driveSubsystem = robotContainer.getDriveSubsystem();
            pigeon = driveSubsystem.getPigeon2();
            driverController = robotContainer.getDriverController();

            SmartDashboard.putString("Controller A button", "");
            SmartDashboard.putString("Pigeon2 Rate", "");
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }

    public void updateDashboard() {
        try{
            SmartDashboard.putString("Controller A button", driverController.getAButton() ? "Pressed" : "");
            SmartDashboard.putString("Pigeon2 Rate", ""+pigeon.getRate());
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }
}
