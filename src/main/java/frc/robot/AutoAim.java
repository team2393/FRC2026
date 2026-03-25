// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.tools.RangeUtil.isBetween;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveOI;
import frc.tools.LookupTable;
import frc.tools.LookupTable.Entry;

/** Subsystem for autimatic aim, spinner and hood adjustment */
public class AutoAim extends SubsystemBase
{
    /** TODO Estimated ball speed [m/s]
     *  This is the 'horizontal' component
     */
    private final double BALL_SPEED = 3.0;

    /** Map of all tags on the field */
    private final AprilTagFieldLayout tags;

    /** Center of each hubs */
    private final Translation2d BLUE_HUB, RED_HUB;

    /** Drivetrain */
    private final SwerveDrivetrain drivetrain;

    private final NetworkTableEntry nt_distance = SmartDashboard.getEntry("HubDistance");
    private final NetworkTableEntry nt_always_config_shooter = SmartDashboard.getEntry("AlwaysConfigShooter");
    private final NetworkTableEntry nt_spinner_setpoint = SmartDashboard.getEntry("SpinnerSetpoint");
    private final NetworkTableEntry nt_hood_setpoint = SmartDashboard.getEntry("HoodSetpoint");

    private final ProfiledPIDController pid = new ProfiledPIDController(5, 1, 0,
                                                    new TrapezoidProfile.Constraints(3*360, 3*360));
    private Translation2d aim_target = null;
    private Pose2d robot_pose, last_pose;
    private Translation2d direction_to_target = null;
    private Entry shooter_settings = null;

    private final static LookupTable settings_table = new LookupTable(
        // distance, speed, hood, deviation
        //       [m], [rpm],  [%], deviation
         1.3,  1785,   55,         0,
                 1.5,  1780,   70,         0,
                 2.0,  1815,   70,         0,
                 2.5,  1770,   85,         0,
                 3.5,  2030,   85,         0,
                 4.0,  2120,   90,         0,
                 4.7,  2250,   90,         0,
                 7.0,  2400,   90,         0,
                 8.0,  2500,   90,         0);

    /** @param tags {@link AprilTagFieldLayout}
     *  @param drivetrain {@link SwerveDrivetrain}
     */
    public AutoAim(AprilTagFieldLayout tags, SwerveDrivetrain drivetrain)
    {
        this.tags = tags;
        this.drivetrain = drivetrain;

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

        // Use PID with -180..180 degrees, enable I below 2 deg error, done when within 1 deg
        pid.enableContinuousInput(-180, 180);
        pid.setIZone(2.0);
        pid.setTolerance(1.0);
        // SmartDashboard.putData("AimToHubPID", pid);

        nt_always_config_shooter.setDefaultBoolean(true);

        last_pose = robot_pose = drivetrain.getPose();
    }

    /** Update robot_pose, identify aim_target */
    private void identifyTarget()
    {
        robot_pose = drivetrain.getPose();

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
        {   // In home area, aim for hub
            if (isBetween(robot_pose.getTranslation().getX(), 0, BLUE_HUB.getX()))
                aim_target = BLUE_HUB;
            else
            {   // Pass to lower or upper half of home area
                double x = 0.5*BLUE_HUB.getX();
                if (robot_pose.getTranslation().getY() < 0.5*tags.getFieldWidth())
                    aim_target = new Translation2d(x, 0.25*tags.getFieldWidth());
                else
                    aim_target = new Translation2d(x, 0.75*tags.getFieldWidth());
            }
        }
        else // Red alliance
        {   // In home area, aim for hub
            if (isBetween(robot_pose.getTranslation().getX(), RED_HUB.getX(), tags.getFieldLength()))
                aim_target = RED_HUB;
            else
            {   // Pass to lower or upper half of home area
                double x = RED_HUB.getX() + 0.5*(tags.getFieldLength()-RED_HUB.getX());
                if (robot_pose.getTranslation().getY() < 0.5*tags.getFieldWidth())
                    aim_target = new Translation2d(x, 0.25*tags.getFieldWidth());
                else
                    aim_target = new Translation2d(x, 0.75*tags.getFieldWidth());
            }
        }
    }

    /** Computes direction_to_target, shooter_settings and optionally applies settings */
    private void computeSettings()
    {
        // How fast are we?
        Translation2d robot_speed = robot_pose.getTranslation()
                                              .minus(last_pose.getTranslation())
                                              .div(TimedRobot.kDefaultPeriod);
        last_pose = robot_pose;

        // Direction from where we are to hub
        direction_to_target = aim_target.minus(robot_pose.getTranslation());
        double distance = direction_to_target.getNorm();

        // Estimate time for ball to travel that distance
        double shot_time = distance / BALL_SPEED;
        // Determine how far robot travels in that time,
        Translation2d robot_travel = robot_speed.times(shot_time);
        // Estimate where hub will appear to be ...
        Translation2d perceived_hub = aim_target.minus(robot_travel);
        // .. and aim for that
        direction_to_target = perceived_hub.minus(robot_pose.getTranslation());

        // Set spinner speed, hood angle, .. based on distance using LookupTable
        shooter_settings = settings_table.lookup(distance);
        if (nt_always_config_shooter.getBoolean(true))
        {
            nt_spinner_setpoint.setDouble(shooter_settings.speed());
            nt_hood_setpoint.setDouble(shooter_settings.hood());
        }

        nt_distance.setDouble(distance);
    }

    private void startAim()
    {
        // Profiled PID needs to start with current measurement (robot heading)
        pid.reset(last_pose.getRotation().getDegrees());
    }

    private void aim()
    {
        // Were settings already applied, or do we do that now?
        if (!nt_always_config_shooter.getBoolean(true))
        {
            nt_spinner_setpoint.setDouble(shooter_settings.speed());
            nt_hood_setpoint.setDouble(shooter_settings.hood());
        }

        double hub_angle = direction_to_target.getAngle().getDegrees();

        // Swerve speeds from controller
        double vx = SwerveOI.getForwardSpeed();
        double vy = SwerveOI.getLeftSpeed();
        double heading = last_pose.getRotation().getDegrees();
        double correction = -heading;
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            correction += 180;
        Translation2d absoluteDirection = new Translation2d(vx, vy).rotateBy(Rotation2d.fromDegrees(correction));
        vx = absoluteDirection.getX();
        vy = absoluteDirection.getY();

        // Rotation to aim for hub
        double vr = pid.calculate(heading, hub_angle);
        // System.out.println("Heading: " + heading +
        //                    " Goal: " + hub_angle +
        //                    " rot: " + vr);

        drivetrain.swerve(vx, vy, Math.toRadians(vr));
    }

    private boolean isDone()
    {
        // We're done once the error is small enough
        return pid.atGoal();
    }

    /** Subsystem continually computes direction and settings,
     *  optionally also applies shooter_settings.
     *
     *  Commands,which run right after subsystem's periodic(),
     *  aim either once or continually
     */
    @Override
    public void periodic()
    {
        identifyTarget();
        computeSettings();
    }

    /** @return Command that aims once */
    public Command aimOnce()
    {
        Command cmd = new Command()
        {
            @Override
            public void initialize()
            {
               startAim();
            }

            @Override
            public void execute()
            {
                aim();
            }

            @Override
            public boolean isFinished()
            {
                return isDone();
            }

            @Override
            public void end(boolean interrupted)
            {
                drivetrain.stop();
            }
        };
        cmd.addRequirements(this, drivetrain);
        return cmd;
    }

    /** @return Command that aims continually */
    public Command aimContinuously()
    {
        Command cmd = new Command()
        {
            @Override
            public void initialize()
            {
               startAim();
            }

            @Override
            public void execute()
            {
                aim();
            }

            @Override
            public void end(boolean interrupted)
            {
                drivetrain.stop();
            }
        };
        cmd.addRequirements(this, drivetrain);
        return cmd;
    }
}
