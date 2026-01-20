// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Command that shows on dashboard if hub is active
 *
 *  Command needs to be started with teleop
 */
public class HubTimer extends Command
{
    private final NetworkTableEntry nt_active = SmartDashboard.getEntry("HubActive");
    private final Timer timer = new Timer();
    private Alliance alliance;

    public HubTimer()
    {
        nt_active.setBoolean(true);
    }

    // Command should be started with teleop
    @Override
    public void initialize()
    {
        // On which side are we playing?
        alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        timer.start();
    }

    @Override
    public void execute()
    {
        // From https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
        // "The alliance will be provided as a single character representing
        //  the color of the alliance whose goal will go inactive first (i.e. ‘R’ = red, ‘B’ = blue).
        //  This alliance’s goal will be active in Shifts 2 and 4."
        String message = DriverStation.getGameSpecificMessage();
        Alliance inactive_first;
        if ("R".equalsIgnoreCase(message))
            inactive_first = Alliance.Red;
        else if ("B".equalsIgnoreCase(message))
            inactive_first = Alliance.Blue;
        else
        {
            // We don't have game data, assume we're in transition period
            // were hub is active, or in practice run were it's always active
            nt_active.setBoolean(true);
            return;
        }

        // How much time has elapsed in teleop?
        double elapsed = timer.get();

        // See https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf 6.4.1
        // In which match timeframe are we?
        // 10 sec transition
        if (elapsed < 10)
            nt_active.setBoolean(true);
        // 25 sec shift 1
        else if (elapsed < 10 + 25)
            nt_active.setBoolean(alliance != inactive_first);
        // 25 sec shift 2
        else if (elapsed < 10 + 25 + 25)
            nt_active.setBoolean(alliance == inactive_first);
        // 25 sec shift 3
        else if (elapsed < 10 + 25 + 25 + 25)
            nt_active.setBoolean(alliance != inactive_first);
        // 25 sec shift 4
        else if (elapsed < 10 + 25 + 25 + 25 + 25)
            nt_active.setBoolean(alliance == inactive_first);
        // then end game
        else
            nt_active.setBoolean(true);
    }
}
