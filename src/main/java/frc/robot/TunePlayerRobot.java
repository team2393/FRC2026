// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.tools.CommandRobotBase;

/** Test */
public class TunePlayerRobot extends CommandRobotBase
{
    Orchestra orch = new Orchestra();

    public TunePlayerRobot()
    {
        orch.loadMusic("tune.chrp");
        // Practice robot
        for (int i=2; i<=8; ++i)
            orch.addInstrument(new TalonFX(i));

        // orch.addInstrument(new TalonFX(RobotMap.FRONT_LEFT_DRIVE));
        // orch.addInstrument(new TalonFX(RobotMap.FRONT_RIGHT_DRIVE));
        // orch.addInstrument(new TalonFX(RobotMap.BACK_RIGHT_DRIVE));
        // orch.addInstrument(new TalonFX(RobotMap.BACK_LEFT_DRIVE));
        // orch.addInstrument(new TalonFX(RobotMap.FRONT_LEFT_ROTATE));
        // orch.addInstrument(new TalonFX(RobotMap.FRONT_RIGHT_ROTATE));
        // orch.addInstrument(new TalonFX(RobotMap.BACK_RIGHT_ROTATE));
        // orch.addInstrument(new TalonFX(RobotMap.BACK_LEFT_ROTATE));
    }



    @Override
    public void teleopInit()
    {
        orch.play();
        // Don't call set... on motor!
    }
}
