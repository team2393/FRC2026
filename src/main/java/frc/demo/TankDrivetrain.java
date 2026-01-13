// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.demo;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorHelper;

/** Tank-type drivetrain */
public class TankDrivetrain extends SubsystemBase
{
    private final TalonFX left_motor  = MotorHelper.createTalonFX(1, false, false, 0.3);
    private final TalonFX right_motor = MotorHelper.createTalonFX(2, false, false, 0.3);
    // XXX Is there a left and right follower?
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.8);
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.kZero, 0.0, 0.0);
    private final Field2d field = new Field2d();
    private double simulated_left_distance  = 0.0;
    private double simulated_right_distance = 0.0;
    private double simulated_heading = 0.0;

    public TankDrivetrain()
    {
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic()
    {
        field.setRobotPose(odometry.getPoseMeters());
    }

    /** @param vx Forward speed [m/s]
     *  @param vr Rotation [deg/s]
     */
    void drive(double vx, double vr)
    {
        double vr_rad = Math.toRadians(vr);
        DifferentialDriveWheelSpeeds wheel_speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(vx, 0, vr_rad));
        if (RobotBase.isSimulation())
        {
            simulated_left_distance  += wheel_speeds.leftMetersPerSecond  * TimedRobot.kDefaultPeriod;
            simulated_right_distance += wheel_speeds.rightMetersPerSecond * TimedRobot.kDefaultPeriod;
            simulated_heading        += vr                                * TimedRobot.kDefaultPeriod;
            odometry.update(Rotation2d.fromDegrees(simulated_heading), simulated_left_distance, simulated_right_distance);
        }
        else
        {
            // TODO Control motors to desired wheel speeds with feed forward and PID
            left_motor.setVoltage(wheel_speeds.leftMetersPerSecond);
            right_motor.setVoltage(wheel_speeds.rightMetersPerSecond);
            // TODO Update odometry from wheel distance sensors and gyro
        }
    }
}
