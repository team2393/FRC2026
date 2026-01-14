// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Fuel hander
 *
 *  Basic idea:
 *
 *  Intake    Storage          Spinner
 *  O O O O   O O O O O O O O
 *  --->--->  --->--->--->--S  >>>>>>>>
 *
 *  Intake can open/close.
 *  Intake and storage have motors for grabber wheels or belt
 *  to move balls/fuel 'O' in.
 *  Sensor 'S' detects when game item O reaches end/top of storage.
 *  Moving the storage motor any further would feed a game item
 *  into the shooter.
 *  In shooter, a spinner motor ejects game pieces.
 */
public class FuelHandler extends SubsystemBase
{
    // XXX For now DO, will turn into solenoid or motor
    private final DigitalOutput open_intake = new DigitalOutput(RobotMap.INTAKE_OPENER);
    private final TalonFX intake_mover = MotorHelper.createTalonFX(RobotMap.INTAKE_MOVER, false, true, 0.3);
    private final TalonFX storage_mover = MotorHelper.createTalonFX(RobotMap.STOREAGE_MOVER, false, true, 0.3);
    private final DigitalInput storage_sensor = new DigitalInput(RobotMap.STORAGE_SENSOR);
    private final Spinner spinner = new Spinner();
    private final NetworkTableEntry nt_belt_voltage = SmartDashboard.getEntry("BeltVoltage");
    private final NetworkTableEntry nt_storage_full = SmartDashboard.getEntry("StorageFull");

    enum States
    {
        /** All motors off, intake closed */
        Idle,
        /** Open intake, move intake & storage until storage is full */
        TakeIn,
        /** Close intake, run spinner up to speed */
        PrepShooting,
        /** Shoot until storage is empty */
        Shoot
    }
    private States state = States.Idle;

    /** Timer used to keep spinner running after last game piece */
    private final Timer after_shot_timer = new Timer();

    /** Visualization */
    private final static Color8Bit BELT_OFF = new Color8Bit(100, 100, 0);
    private final static Color8Bit BELT_ON = new Color8Bit(255, 255, 0);
    private final static Color8Bit SPINNER_OFF = new Color8Bit(100, 0, 0);
    private final static Color8Bit SPINNER_ON = new Color8Bit(255, 0, 0);
    private final MechanismLigament2d intake, storage, shooter;

    public FuelHandler()
    {
        nt_belt_voltage.setDefaultDouble(3.0);

        // Visualization
        Mechanism2d mech = new Mechanism2d(1.0, 1.0);

        mech.getRoot("left", 0, 0.2).append(new MechanismLigament2d("base", 0.8, 0, 10, new Color8Bit(100, 100, 100)));

        MechanismRoot2d right = mech.getRoot("right", 0.8, 0.2);
        right.append(intake  = new MechanismLigament2d("intake",  0.2,  90, 10, BELT_OFF));
        right.append(storage = new MechanismLigament2d("storage", 0.6, 170, 10, BELT_OFF));

        storage.append(shooter = new MechanismLigament2d("shooter", 0.2, -70, 10, SPINNER_OFF));

        SmartDashboard.putData("FuelHandler", mech);
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
        boolean run_intake = false;
        boolean run_storage = false;
        boolean run_spinner = false;
        boolean storage_full = storage_sensor.get();
        nt_storage_full.setBoolean(storage_full);

        if (state == States.Idle)
        {
            run_intake = false;
            run_storage = false;
            run_spinner = false;
        }
        else if (state == States.TakeIn)
        {
            run_intake = true;
            run_storage = true;
            run_spinner = false;
            if (storage_full)
                state = States.Idle;
        }
        else if (state == States.PrepShooting)
        {
            run_intake = false;
            run_storage = false;
            run_spinner = true;
            if (spinner.isAtSetpoint())
            {
                after_shot_timer.stop();
                state = States.Shoot;
            }
        }
        else if (state == States.Shoot)
        {
            run_intake = false;
            run_storage = true;
            run_spinner = true;
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

        boolean blink_on_off = (System.currentTimeMillis()/200) % 2 == 0;

        if (run_intake)
        {
            open_intake.set(true);
            intake_mover.setVoltage(nt_belt_voltage.getDouble(0));
            intake.setAngle(-20);
            intake.setColor(blink_on_off ? BELT_ON : BELT_OFF);
        }
        else
        {
            open_intake.set(false);
            intake_mover.setVoltage(0);
            intake.setAngle(90);
            intake.setColor(BELT_OFF);
        }

        if (run_storage)
        {
            storage_mover.setVoltage(nt_belt_voltage.getDouble(0));
            storage.setColor(blink_on_off ? BELT_ON : BELT_OFF);
        }
        else
        {
            storage_mover.setVoltage(0);
            storage.setColor(BELT_OFF);
        }

        if (run_spinner)
        {
            spinner.runAtSpeedSetpoint();
            shooter.setColor(blink_on_off ? SPINNER_ON : SPINNER_OFF);
        }
        else
        {
            spinner.setVoltage(0);
            shooter.setColor(SPINNER_OFF);
        }
    }
}
