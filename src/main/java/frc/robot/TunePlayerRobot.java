// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.io.File;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Filesystem;
import frc.tools.CommandRobotBase;

/** Test */
public class TunePlayerRobot extends CommandRobotBase
{
    Orchestra orch = new Orchestra();

    public TunePlayerRobot()
    {
        // orch.loadMusic(new File(Filesystem.getDeployDirectory(), "tune.chrp").getAbsolutePath());
        orch.loadMusic("tune.chrp");
        orch.addInstrument(new TalonFX(RobotMap.FRONT_LEFT_DRIVE));
        orch.addInstrument(new TalonFX(RobotMap.FRONT_RIGHT_DRIVE));
        orch.addInstrument(new TalonFX(RobotMap.BACK_RIGHT_DRIVE));
        orch.addInstrument(new TalonFX(RobotMap.BACK_LEFT_DRIVE));
        orch.addInstrument(new TalonFX(RobotMap.FRONT_LEFT_ROTATE));
        orch.addInstrument(new TalonFX(RobotMap.FRONT_RIGHT_ROTATE));
        orch.addInstrument(new TalonFX(RobotMap.BACK_RIGHT_ROTATE));
        orch.addInstrument(new TalonFX(RobotMap.BACK_LEFT_ROTATE));
    }

    @Override
    public void teleopInit()
    {
        orch.play();
        // Don't call set... on motor!
    }
}
