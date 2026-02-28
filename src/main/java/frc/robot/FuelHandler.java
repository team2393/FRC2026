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
import frc.robot.Feeder.Mode;
import frc.tools.KeepOnFilter;

/** Fuel hander state machine
 *
 *  Basic idea:
 *
 *  Intake    Storage          Feeder Spinner
 *  O O O O   O O O O O O O O O O O O
 *  --->--->  --->--->--->--->--->--S  >>>>>>>>
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
    // TODO private final Intake intake = new Intake();

    private final Storage storage = new Storage();

    private final Feeder feeder = new Feeder();
    private final KeepOnFilter keep_feeder = new KeepOnFilter(1.0);
    private final NetworkTableEntry nt_feeder_full = SmartDashboard.getEntry("FeederFull");

    private final Spinner spinner = new Spinner();
    private final NetworkTableEntry nt_always_spin  = SmartDashboard.getEntry("AlwaysSpin");

    enum States
    {
        /** All motors off, intake closed */
        Idle,
        /** Open intake, move storage and feeder until game piece in feeder */
        TakeIn,

        /** Intake closed; move storage to push game pieces up to feeder;
         *  run feeder unless game piece in feeder
         */
        Store,
        /** Close intake, run spinner up to speed */
        PrepShooting,
        /** Shoot until storage is empty */
        Shoot
    }
    private States state = States.Idle;

    /** Keep spinner running a little longer after last game piece has been detected */
    private final Debouncer after_shot_delay = new Debouncer(1.0);

    /** Visualization */
    private final static Color8Bit MOVE_OFF = new Color8Bit(100, 100, 0);
    private final static Color8Bit MOVE_ON = new Color8Bit(255, 255, 0);
    private final static Color8Bit SPINNER_OFF = new Color8Bit(100, 0, 0);
    private final static Color8Bit SPINNER_ON = new Color8Bit(255, 0, 0);
    private final MechanismLigament2d vis_intake, vis_storage, vis_shooter;

    public FuelHandler()
    {
        nt_always_spin.setDefaultBoolean(false);

        // Visualization
        Mechanism2d mech = new Mechanism2d(1.0, 1.0);

        mech.getRoot("left", 0, 0.2).append(new MechanismLigament2d("base", 0.8, 0, 10, new Color8Bit(100, 100, 100)));

        MechanismRoot2d right = mech.getRoot("right", 0.8, 0.2);
        right.append(vis_intake  = new MechanismLigament2d("intake",  0.2,  90, 10, MOVE_OFF));
        right.append(vis_storage = new MechanismLigament2d("storage", 0.6, 170, 10, MOVE_OFF));

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
        return new InstantCommand(() -> state = States.Store);
    }

    /** @return Command that toggles take in, store */
    public Command toggleIntake()
    {
        return new InstantCommand(() ->
        {
            if (state == States.TakeIn)
                state = States.Store;
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
        // Assume all should be off, then turn on what's necessary based on state
        boolean run_intake = false;
        boolean run_storage = false;
        Feeder.Mode feeder_mode = Feeder.Mode.OFF;
        boolean run_spinner = false;

        // As we push balls out, there can be brief gaps between balls.
        // Bridge those to keep feeder on for a little longer
        boolean feeder_full = keep_feeder.calculate(feeder.haveBall());
        nt_feeder_full.setBoolean(feeder_full);

        if (state == States.Idle)
        {
            // Leave all off
        }
        if (state == States.TakeIn)
        {
            if (feeder_full)
                state = States.Store;
            else
            {
                run_intake = true;
                run_storage = true;
                feeder_mode = Feeder.Mode.FEED;
            }
        }
        if (state == States.Store)
        {
            run_storage = true;
            feeder_mode = feeder_full ? Mode.OFF : Mode.FEED;
        }
        if (state == States.PrepShooting)
        {
            run_storage = true;
            feeder_mode = feeder_full ? Mode.OFF : Mode.FEED;
            run_spinner = true;
            if (spinner.isAtSetpoint())
            {
                // Reset debouncer
                after_shot_delay.calculate(false);
                state = States.Shoot;
            }
        }
        if (state == States.Shoot)
        {
            run_storage = true;
            feeder_mode = Mode.SHOOT;
            run_spinner = true;

            // All game items gone? Keep spinner running to make sure
            // all items are really shot.
            if (after_shot_delay.calculate(!feeder_full))
                state = States.Store;
        }

        boolean blink_on_off = (System.currentTimeMillis()/200) % 2 == 0;

        if (run_intake)
        {
            // TODO intake.open(true);
            vis_intake.setAngle(-20);
            vis_intake.setColor(blink_on_off ? MOVE_ON : MOVE_OFF);
        }
        else
        {
            // TODO intake.open(false);
            vis_intake.setAngle(90);
            vis_intake.setColor(MOVE_OFF);
        }

        if (run_storage)
        {
            storage.run(true);
            vis_storage.setColor(blink_on_off ? MOVE_ON : MOVE_OFF);
        }
        else
        {
            storage.run(false);
            vis_storage.setColor(MOVE_OFF);
        }

        feeder.run(feeder_mode);

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
