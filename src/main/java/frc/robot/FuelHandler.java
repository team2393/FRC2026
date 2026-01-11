// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Fuel hander
 *
 *  Basic idea:
 *
 *  intake/storage            Spinner
 *  X X X X X X X X X X X X
 *  --->--->--->--->--->--S>  >>>>>>>>
 *
 *  'intake' or 'storage' motor moves grabber wheels or belt
 *  to move items X in.
 *  A sensor S detects when game item X reaches end/top of storage.
 *  Moving the 'storage' motor any further would feed a game item
 *  into the shooter.
 *  In shooter, a spinner motor ejects game pieces.
 */
public class FuelHandler extends SubsystemBase
{
    private final TalonFX storage_mover = MotorHelper.createTalonFX(RobotMap.STOREAGE_MOVER, false, true, 0.3);
    private final DigitalInput storage_sensor = new DigitalInput(RobotMap.STORAGE_SENSOR);
    private final Spinner spinner = new Spinner();
    private final NetworkTableEntry nt_storage_voltage = SmartDashboard.getEntry("StorageVoltage");
    private final NetworkTableEntry nt_storage_full = SmartDashboard.getEntry("StorageFull");

    enum States
    {
        /** All motors off */
        Idle,
        /** Move intake/storage until storage is full */
        TakeIn,
        /** Run spinner up to speed */
        PrepShooting,
        /** Shoot until storage is empty */
        Shoot
    }
    private States state = States.Idle;

    /** Timer used to keep spinner running after last game piece */
    private final Timer after_shot_timer = new Timer();

    public FuelHandler()
    {
        nt_storage_voltage.setDefaultDouble(3.0);
    }

    /** @return Command that starts taking game pieces in */
    public Command take_in()
    {
        return new InstantCommand(() -> state = States.TakeIn);
    }

    /** @return Command that starts shooting */
    public Command shoot()
    {
        return new InstantCommand(() -> state = States.PrepShooting);
    }

    @Override
    public void periodic()
    {
        boolean storage_full = storage_sensor.get();
        nt_storage_full.setBoolean(storage_full);

        if (state == States.Idle)
        {
            storage_mover.setVoltage(0);
            spinner.setVoltage(0);
        }
        else if (state == States.TakeIn)
        {
            storage_mover.setVoltage(nt_storage_voltage.getDouble(0));
            spinner.setVoltage(0);
            if (storage_full)
                state = States.Idle;
        }
        else if (state == States.PrepShooting)
        {
            storage_mover.setVoltage(0);
            spinner.runAtSpeedSetpoint();
            if (spinner.isAtSetpoint())
            {
                after_shot_timer.stop();
                state = States.Shoot;
            }
        }
        else if (state == States.Shoot)
        {
            storage_mover.setVoltage(nt_storage_voltage.getDouble(0));
            spinner.runAtSpeedSetpoint();
            if (!storage_full)
            {
                // All game items gone? Keep spinner running to make sure
                // all items are really shot.
                // Start timer and let it run for a little
                if (!after_shot_timer.isRunning())
                    after_shot_timer.start();
                else if (after_shot_timer.hasElapsed(1.0))
                    state = States.Idle;
            }
        }
    }
}
