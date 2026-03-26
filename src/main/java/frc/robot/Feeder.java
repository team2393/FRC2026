// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Feeder */
public class Feeder
{
    private final TalonFX motor = MotorHelper.createTalonFX(RobotMap.FEEDER, false, false, 0, 40.0);
    private final DigitalInput feeder_sensor = new DigitalInput(RobotMap.FEEDER_SENSOR);
    private final NetworkTableEntry nt_have_ball = SmartDashboard.getEntry("BallInFeeder");
    private final NetworkTableEntry nt_intake_voltage = SmartDashboard.getEntry("FeederIntakeVoltage");
    private final NetworkTableEntry nt_shooting_voltage = SmartDashboard.getEntry("FeederShootingVoltage");

    public static enum Mode
    {
        /** Off */
        OFF,
        /** Run at feeding speed, stop when ball is detected */
        FEED,
        /** Run at shooting speed */
        SHOOT
    }

    public Feeder()
    {
        nt_intake_voltage.setDefaultDouble(5.0);
        nt_shooting_voltage.setDefaultDouble(6.0);
    }

    /** Calling this has side effect of updating the NT indicator
     *  @return Do we have a ball?
     */
    public boolean haveBall()
    {
        final boolean have_ball = feeder_sensor.get();
        nt_have_ball.setBoolean(have_ball);
        return have_ball;
    }

    public void run(Mode mode)
    {
        if (mode == Mode.OFF)
            motor.setVoltage(0);
        else if (mode == Mode.SHOOT)
            motor.setVoltage(nt_shooting_voltage.getDouble(0));
        else
            motor.setVoltage(haveBall() ? 0
                                        : nt_intake_voltage.getDouble(0));
    }
}
