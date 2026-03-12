// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
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
 */
public class FuelHandler extends SubsystemBase
{
    private final Intake intake = new Intake();

    private final Storage storage = new Storage();

    private final Feeder feeder = new Feeder();
    private final KeepOnFilter keep_feeder = new KeepOnFilter(1.0);
    private final NetworkTableEntry nt_feeder_full = SmartDashboard.getEntry("FeederFull");

    private final Spinner spinner = new Spinner();
    private final NetworkTableEntry nt_always_spin  = SmartDashboard.getEntry("AlwaysSpin");

    /** Intake and rest are mostly kept separate
     *  Intake can open/close, running intake feeder while open.
     *  Intake may remain open for a long time because closing intake
     *  would also close the hopper and reduce number of balls
     *  that can be stored
     */
    enum IntakeState
    {
        Closed,
        Open
    }

    /** Shooter state covers rest: Storage, feeder, spinner */
    enum ShooterState
    {
        /** All off */
        Idle,
        /** Store game pieces */
        Storing,
        /** Run spinner up to speed */
        PrepShooting,
        /** Shoot until feeder is empty */
        Shoot
    }

    private IntakeState intake_state = IntakeState.Closed;
    private ShooterState shooter_state = ShooterState.Idle;

    /** Keep spinner running a little longer after last game piece has been detected */
    private final Debouncer after_shot_delay = new Debouncer(1.0);

    /** Visualization */
    private final static Color8Bit MOVE_OFF = new Color8Bit(100, 100, 0);
    private final static Color8Bit MOVE_ON = new Color8Bit(255, 255, 0);
    private final static Color8Bit SPINNER_OFF = new Color8Bit(100, 0, 0);
    private final static Color8Bit SPINNER_ON = new Color8Bit(255, 0, 0);
    private final MechanismLigament2d vis_intake, vis_storage, vis_feeder, vis_shooter;

    public FuelHandler()
    {
        nt_always_spin.setDefaultBoolean(false);

        // Visualization
        Mechanism2d mech = new Mechanism2d(1.0, 1.0);

        mech.getRoot("left", 0, 0.2).append(new MechanismLigament2d("base", 0.8, 0, 10, new Color8Bit(100, 100, 100)));

        MechanismRoot2d right = mech.getRoot("right", 0.8, 0.2);
        right.append(vis_intake  = new MechanismLigament2d("intake",  0.2,  90, 10, MOVE_OFF));
        right.append(vis_storage = new MechanismLigament2d("storage", 0.6, 170, 10, MOVE_OFF));

        vis_storage.append(vis_feeder = new MechanismLigament2d("feeder", 0.1, -70, 10, MOVE_OFF));
        vis_feeder.append(vis_shooter = new MechanismLigament2d("shooter", 0.1, 0, 10, SPINNER_OFF));

        SmartDashboard.putData("FuelHandler", mech);
    }

    /** @return Command that opens intake */
    public Command openIntake()
    {
        return new InstantCommand(() -> intake_state = IntakeState.Open);
    }

    /** @return Command that closes intake */
    public Command closeIntake()
    {
        return new InstantCommand(() -> intake_state = IntakeState.Closed);
    }

    /** @return Command that toggles intake open/close */
    public Command toggleIntake()
    {
        return new InstantCommand(() ->
        {
            if (intake_state == IntakeState.Closed)
                intake_state = IntakeState.Open;
            else
                intake_state = IntakeState.Closed;
        });
    }

    /** @return Command that starts/stops shooting */
    public Command toggleShooter()
    {
        return new InstantCommand(() ->
        {
            if (shooter_state == ShooterState.PrepShooting ||
                shooter_state == ShooterState.Shoot)
                shooter_state = ShooterState.Storing;
            else
                shooter_state = ShooterState.PrepShooting;
        });
    }

    /** @return Command that starts shooting and waits until done */
    public Command shoot()
    {
        return new Command()
        {
            @Override
            public void initialize()
            {
                shooter_state = ShooterState.PrepShooting;
            }

            @Override
            public boolean isFinished()
            {
                return shooter_state == ShooterState.Storing;
            }
        };
    }

    /** @return Command that stops shooting */
    public Command store()
    {
        return new InstantCommand()
        {
            @Override
            public void initialize()
            {
                shooter_state = ShooterState.Storing;
            }
        };
    }

    @Override
    public void periodic()
    {
        // Assume all should be off, then turn on what's necessary based on state
        boolean run_storage = false;
        Feeder.Mode feeder_mode = Feeder.Mode.OFF;
        boolean run_spinner = false;

        // As we push balls out, there can be brief gaps between balls.
        // Bridge those to keep feeder on for a little longer
        boolean feeder_full = keep_feeder.calculate(feeder.haveBall());
        nt_feeder_full.setBoolean(feeder_full);

        if (shooter_state == ShooterState.Idle)
        {
            if (intake_state == IntakeState.Open)
                shooter_state = ShooterState.Storing;
        }
        if (shooter_state == ShooterState.Storing)
        {
            if (!feeder_full)
            {
                run_storage = true;
                feeder_mode = Feeder.Mode.FEED;
            }
        }
        if (shooter_state == ShooterState.PrepShooting)
        {
            run_storage = true;
            if (!feeder_full)
            {
                feeder_mode = Feeder.Mode.FEED;
            }
            run_spinner = true;
            if (spinner.isAtSetpoint())
            {
                // Reset debouncer
                after_shot_delay.calculate(false);
                shooter_state = ShooterState.Shoot;
            }
        }
        if (shooter_state == ShooterState.Shoot)
        {
            run_storage = true;
            feeder_mode = Mode.SHOOT;
            run_spinner = true;

            // All game items gone? Keep spinner running to make sure
            // all items are really shot.
            if (after_shot_delay.calculate(!feeder_full))
                shooter_state = ShooterState.Storing;
        }

        boolean blink_on_off = (System.currentTimeMillis()/200) % 2 == 0;

        // Open or close intake
        if (intake_state == IntakeState.Open)
        {
            intake.open(true);
            vis_intake.setAngle(-20);
            vis_intake.setColor(blink_on_off ? MOVE_ON : MOVE_OFF);

            // When disabled, we tend to move the arm back up into the initial configuration.
            // In case the state machine was still "Open", correct that
            if (DriverStation.isDisabled() && intake.getAngle() > 100)
            {
                intake_state = IntakeState.Closed;
                System.out.println("Setting intake state to closed");
            }
        }
        else
        {
            intake.open(false, shooter_state == ShooterState.PrepShooting  ||  shooter_state == ShooterState.Shoot);
            vis_intake.setAngle(90);
            vis_intake.setColor(MOVE_OFF);
        }

        // Run storage?
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

        // Feeder?
        feeder.run(feeder_mode);
        if (feeder_mode == Mode.OFF)
            vis_feeder.setColor(MOVE_OFF);
        else
            vis_feeder.setColor(blink_on_off ? MOVE_ON : MOVE_OFF);

        // Spinner?
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
