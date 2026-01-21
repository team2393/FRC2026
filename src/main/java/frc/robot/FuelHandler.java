// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.tools.KeepOnFilter;

/** Fuel hander state machine
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
 *
 *  For now shoots all game pieces.
 *  To stop shooing after one has been ejected and thus allow
 *  shooting one-by-one, we'd ideally have another sensor
 *  that detects game piece in shooter, so we stop
 *  feeding from storage.
 */
public class FuelHandler extends SubsystemBase
{
    private final Intake intake = new Intake();
    private final TalonFX storage_mover = MotorHelper.createTalonFX(RobotMap.STOREAGE_MOVER, false, true, 0.3);
    private final DigitalInput storage_sensor = new DigitalInput(RobotMap.STORAGE_SENSOR);
    private final KeepOnFilter keep_storarge = new KeepOnFilter(1.0);
    private final Spinner spinner = new Spinner();
    private final NetworkTableEntry nt_belt_voltage = SmartDashboard.getEntry("BeltVoltage");
    private final NetworkTableEntry nt_storage_full = SmartDashboard.getEntry("StorageFull");
    private final NetworkTableEntry nt_always_spin  = SmartDashboard.getEntry("AlwaysSpin");

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

    /** Keep spinner running a little longer after last game piece has been detected */
    private final Debouncer after_shot_delay = new Debouncer(1.0);

    /** Visualization */
    private final static Color8Bit BELT_OFF = new Color8Bit(100, 100, 0);
    private final static Color8Bit BELT_ON = new Color8Bit(255, 255, 0);
    private final static Color8Bit SPINNER_OFF = new Color8Bit(100, 0, 0);
    private final static Color8Bit SPINNER_ON = new Color8Bit(255, 0, 0);
    private final MechanismLigament2d vis_intake, vis_storage, vis_shooter;

    public FuelHandler()
    {
        nt_belt_voltage.setDefaultDouble(3.0);
        nt_always_spin.setDefaultBoolean(false);

        // Visualization
        Mechanism2d mech = new Mechanism2d(1.0, 1.0);

        mech.getRoot("left", 0, 0.2).append(new MechanismLigament2d("base", 0.8, 0, 10, new Color8Bit(100, 100, 100)));

        MechanismRoot2d right = mech.getRoot("right", 0.8, 0.2);
        right.append(vis_intake  = new MechanismLigament2d("intake",  0.2,  90, 10, BELT_OFF));
        right.append(vis_storage = new MechanismLigament2d("storage", 0.6, 170, 10, BELT_OFF));

        vis_storage.append(vis_shooter = new MechanismLigament2d("shooter", 0.2, -70, 10, SPINNER_OFF));

        SmartDashboard.putData("FuelHandler", mech);
    }

    /** @return Command that starts taking game pieces in */
    public Command openIntake()
    {
        return new InstantCommand(() -> state = States.TakeIn);
    }

    /** @return Command that closes the intake */
    public Command closeIntake()
    {
        return new InstantCommand(() -> state = States.Idle);
    }

    /** @return Command that toggles take in, idle */
    public Command toggleIntake()
    {
        return new InstantCommand(() ->
        {
            if (state == States.TakeIn)
                state = States.Idle;
            else
                state = States.TakeIn;
        });
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
        // As we push balls out, gaps between balls suggest
        // there's nothing in storage, so if we detect a ball,
        // keep 'storage_full' on for a little longer
        boolean storage_full = keep_storarge.calculate(storage_sensor.get());
        nt_storage_full.setBoolean(storage_full);

        if (state == States.Idle)
        {
            // Leave all off
        }
        else if (state == States.TakeIn)
        {
            run_intake = true;
            run_storage = true;
            if (storage_full)
                state = States.Idle;
        }
        else if (state == States.PrepShooting)
        {
            run_spinner = true;
            if (spinner.isAtSetpoint())
            {
                // Reset debouncer
                after_shot_delay.calculate(false);
                state = States.Shoot;
            }
        }
        else if (state == States.Shoot)
        {
            run_storage = true;
            run_spinner = true;

            // All game items gone? Keep spinner running to make sure
            // all items are really shot.
            if (after_shot_delay.calculate(!storage_full))
                state = States.Idle;
        }

        boolean blink_on_off = (System.currentTimeMillis()/200) % 2 == 0;

        if (run_intake)
        {
            intake.open(true);
            vis_intake.setAngle(-20);
            vis_intake.setColor(blink_on_off ? BELT_ON : BELT_OFF);
        }
        else
        {
            intake.open(false);
            vis_intake.setAngle(90);
            vis_intake.setColor(BELT_OFF);
        }

        if (run_storage)
        {
            storage_mover.setVoltage(nt_belt_voltage.getDouble(0));
            vis_storage.setColor(blink_on_off ? BELT_ON : BELT_OFF);
        }
        else
        {
            storage_mover.setVoltage(0);
            vis_storage.setColor(BELT_OFF);
        }

        if (run_spinner  ||  nt_always_spin.getBoolean(false))
        {
            spinner.runAtSpeedSetpoint();
            vis_shooter.setColor(blink_on_off ? SPINNER_ON : SPINNER_OFF);
        }
        else
        {
            spinner.setVoltage(0);
            vis_shooter.setColor(SPINNER_OFF);
        }
    }
}
