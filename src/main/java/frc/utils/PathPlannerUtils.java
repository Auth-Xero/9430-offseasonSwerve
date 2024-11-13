package frc.utils;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforward;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class PathPlannerUtils {

    private DriveSubsystem subsystem;
    private Pigeon2 pigeon;
    private RobotConfig config;
    private Pose2d prevPose2d;
    private Pose2d latestPose2d;
    private double prevTimestamp;
    private double latestTimestamp;
    private SwerveDriveOdometry odometry;

    public PathPlannerUtils(DriveSubsystem subsystem, Pigeon2 pigeon) {
        this.subsystem = subsystem;
        this.pigeon = pigeon;

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        config = null;
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                subsystem // Reference to this subsystem to set requirements
        );

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        prevPose2d = latestPose2d;
        latestPose2d = pose;
        prevTimestamp = latestTimestamp;
        latestTimestamp = Timer.getFPGATimestamp();
        pigeon.reset();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {


        double dt = latestTimestamp - prevTimestamp;
        double dx = (latestPose2d.getX() - prevPose2d.getX()) / dt;
        double dy = (latestPose2d.getY() - prevPose2d.getY()) / dt;
        double da = (latestPose2d.getRotation().getRadians() - prevPose2d.getRotation().getRadians()) / dt;

        return new ChassisSpeeds(dx, dy, da);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {}


    
}
