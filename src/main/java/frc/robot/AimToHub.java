// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.swervelib.SwerveDrivetrain;

/** Command that aims at hub
 *
 *  Aims robot at hub based on odometry,
 *  which is ideally updated from camera info.
 */
public class AimToHub extends Command
{
    private final Translation2d BLUE_HUB;
    private final Translation2d RED_HUB;
    private final SwerveDrivetrain drivetrain;
    private final NetworkTableEntry nt_distance = SmartDashboard.getEntry("HubDistance");
    private final ProfiledPIDController pid = new ProfiledPIDController(5, 1, 0,
                                                    new TrapezoidProfile.Constraints(3*360, 3*360));
    private Translation2d hub = null;

    public AimToHub(AprilTagFieldLayout tags, SwerveDrivetrain drivetrain)
    {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        // Center of blue, red hub is between tags ... and ...
        var a = tags.getTagPose(19).get().getTranslation();
        var b = tags.getTagPose(25).get().getTranslation();
        BLUE_HUB = a.interpolate(b, 0.5).toTranslation2d();

        a = tags.getTagPose(9).get().getTranslation();
        b = tags.getTagPose(3).get().getTranslation();
        RED_HUB = a.interpolate(b, 0.5).toTranslation2d();

        // Use PID with -180..180 degrees, enable I below 2 deg error, done when within 1 deg
        pid.enableContinuousInput(-180, 180);
        pid.setIZone(2.0);
        pid.setTolerance(1.0);
        // SmartDashboard.putData("AimToHubPID", pid);
    }

    @Override
    public void initialize()
    {
        // Use "our" hub.
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            hub = RED_HUB;
        else
            hub = BLUE_HUB;

        // Profiled PID needs to start with current measurement (robot heading)
        pid.reset(drivetrain.getPose().getRotation().getDegrees());
    }

    @Override
    public void execute()
    {
        // Where are we?
        Pose2d robot_pose = drivetrain.getPose();

        // Direction from where we are to hub
        Translation2d direction = hub.minus(robot_pose.getTranslation());
        double angle = direction.getAngle().getDegrees();
        double distance = direction.getNorm();

        double vr = pid.calculate(robot_pose.getRotation().getDegrees(), angle);
        // System.out.println("Heading: " + robot_pose.getRotation().getDegrees() +
        //                    " Goal: "+angle +
        //                    " rot: " + vr);
        drivetrain.swerve(0, 0, Math.toRadians(vr));

        // XXX Set spinner speed, hood angle, .. based on distance using LookupTable
        nt_distance.setDouble(distance);
    }

    @Override
    public boolean isFinished()
    {
        // We're done once the error is small enough
        return pid.atGoal();
    }
}
