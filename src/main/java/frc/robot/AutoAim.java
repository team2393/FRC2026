// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveOI;
import frc.tools.LookupTable;
import frc.tools.LookupTable.Entry;

/** Command that aims at hub
 *
 *  Aims robot at hub based on odometry,
 *  which is ideally updated from camera info.
 */
public class AutoAim extends Command
{
    /** Estimated ball speed [m/s] */
    private final double BALL_SPEED = 3.0;
    private final AprilTagFieldLayout tags;
    private final Translation2d BLUE_HUB;
    private final Translation2d RED_HUB;
    private final SwerveDrivetrain drivetrain;
    private final boolean absolute;
    private final NetworkTableEntry nt_distance = SmartDashboard.getEntry("HubDistance");
    private final ProfiledPIDController pid = new ProfiledPIDController(5, 1, 0,
                                                    new TrapezoidProfile.Constraints(3*360, 3*360));
    private Translation2d aim_target = null;
    private Pose2d last_pose = null;

    private final static LookupTable settings_table = new LookupTable(
        // distance, speed, hood, deviation
        //       [m], [rpm],  [%], deviation
         1.3,  1800,   55,         0,
                 1.5,  1800,   70,         0,
                 2.0,  1850,   70,         0,
                 2.5,  1800,   90,         0,
                 3.5,  2050,   85,         0,
                 4.0,  2250,   90,         0);

    /** @param tags {@link AprilTagFieldLayout}
     *  @param drivetrain {@link SwerveDrivetrain}
     */
    public AutoAim(AprilTagFieldLayout tags, SwerveDrivetrain drivetrain)
    {
        this(tags, drivetrain, true);
    }

    /** @param tags {@link AprilTagFieldLayout}
     *  @param drivetrain {@link SwerveDrivetrain}
     *  @param absolute Absolute drive mode?
     */
    public AutoAim(AprilTagFieldLayout tags, SwerveDrivetrain drivetrain, boolean absolute)
    {
        this.tags = tags;

        // Center of blue, red hub is between tags ... and ...
        // ID,X,Y,Z,Z-Rotation,X-Rotation
        // 20,205.873,158.844,44.25,  0,0
        // 26,158.341,158.844,44.25,180,0
        var a = tags.getTagPose(20).get().getTranslation();
        var b = tags.getTagPose(26).get().getTranslation();
        BLUE_HUB = a.interpolate(b, 0.5).toTranslation2d();

        // ID,X,Y,Z,Z-Rotation,X-Rotation
        //  4,445.349,158.844,44.25,180,0
        // 10,492.881,158.844,44.25,  0,0
        a = tags.getTagPose(4).get().getTranslation();
        b = tags.getTagPose(10).get().getTranslation();
        RED_HUB = a.interpolate(b, 0.5).toTranslation2d();

        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        this.absolute = absolute;

        // Use PID with -180..180 degrees, enable I below 2 deg error, done when within 1 deg
        pid.enableContinuousInput(-180, 180);
        pid.setIZone(2.0);
        pid.setTolerance(1.0);
        // SmartDashboard.putData("AimToHubPID", pid);
    }

    /** @param coord Position to check
     *  @param low Low end of range
     *  @param high High end of range
     *  @return Is coordinate within the range?
     */
    private boolean isBetween(double coord, double low, double high)
    {
        if (low > high)
            throw new RuntimeException("Invalid range");
        return low <= coord  &&  coord <= high;
    }

    @Override
    public void initialize()
    {
        last_pose = drivetrain.getPose();

        if (isBetween(last_pose.getTranslation().getX(), 0, BLUE_HUB.getX()))
            aim_target = BLUE_HUB;
        else if (isBetween(last_pose.getTranslation().getX(), RED_HUB.getX(), tags.getFieldLength()))
            aim_target = RED_HUB;
        else if (isBetween(last_pose.getTranslation().getX(), BLUE_HUB.getX(), 0.5*tags.getFieldLength()))
        {
            if (last_pose.getTranslation().getY() < 0.5*tags.getFieldWidth())
                aim_target = new Translation2d(0.5*BLUE_HUB.getX(), 0.25*tags.getFieldWidth());
            else
                aim_target = new Translation2d(0.5*BLUE_HUB.getX(), 0.75*tags.getFieldWidth());
        }
        else
        {
            double x = RED_HUB.getX() + 0.5*(tags.getFieldLength()-RED_HUB.getX());
            if (last_pose.getTranslation().getY() < 0.5*tags.getFieldWidth())
                aim_target = new Translation2d(x, 0.25*tags.getFieldWidth());
            else
                aim_target = new Translation2d(x, 0.75*tags.getFieldWidth());
        }

        // Profiled PID needs to start with current measurement (robot heading)
        pid.reset(last_pose.getRotation().getDegrees());
    }

    @Override
    public void execute()
    {
        // Where are we, how fast are we?
        Pose2d robot_pose = drivetrain.getPose();
        Translation2d robot_speed = robot_pose.getTranslation()
                                              .minus(last_pose.getTranslation())
                                              .div(TimedRobot.kDefaultPeriod);
        last_pose = robot_pose;

        // Direction from where we are to hub
        Translation2d direction = aim_target.minus(robot_pose.getTranslation());
        double distance = direction.getNorm();
        // Estimate time for ball to travel that distance
        double shot_time = distance / BALL_SPEED;
        // Determine how far robot travels in that time,
        Translation2d robot_travel = robot_speed.times(shot_time);
        // Estimate where hub will appear to be ...
        Translation2d perceived_hub = aim_target.minus(robot_travel);
        // .. and aim for that
        direction = perceived_hub.minus(robot_pose.getTranslation());
        // Where do we have to point?
        double hub_angle = direction.getAngle().getDegrees();

        // Swerve speeds from controller
        double vx = SwerveOI.getForwardSpeed();
        double vy = SwerveOI.getLeftSpeed();
        if (absolute)
        {
            double heading = robot_pose.getRotation().getDegrees();
            double correction = -heading;
            if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
                correction += 180;
            Translation2d absoluteDirection = new Translation2d(vx, vy).rotateBy(Rotation2d.fromDegrees(correction));
            vx = absoluteDirection.getX();
            vy = absoluteDirection.getY();
        }

        // Rotation to aim for hub
        double vr = pid.calculate(robot_pose.getRotation().getDegrees(), hub_angle);
        // System.out.println("Heading: " + robot_pose.getRotation().getDegrees() +
        //                    " Goal: " + hub_angle +
        //                    " rot: " + vr);

        drivetrain.swerve(vx, vy, Math.toRadians(vr));

        nt_distance.setDouble(distance);

        // Set spinner speed, hood angle, .. based on distance using LookupTable
        Entry settings = settings_table.lookup(distance);
        SmartDashboard.putNumber("SpinnerSetpoint", settings.speed());
        SmartDashboard.putNumber("HoodSetpoint", settings.hood());
    }

    @Override
    public boolean isFinished()
    {
        // We're done once the error is small enough
        return pid.atGoal();
    }
}
