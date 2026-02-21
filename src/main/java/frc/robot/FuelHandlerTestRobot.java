// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.tools.CommandRobotBase;

/** Fuel handler test
 *
 *  In teleop, press a
 *  Intake opens and pulls game pieces in until
 *  game piece is detected by 'StorageFull'.
 *  Might also press a while taking in to stop/close.
 *
 *  Press y to start shooting.
 *  When shooter speed reaches setpoint,
 *  game pieces will be moved out
 *  until 'StorageFull' indicates no more.
 *  Shooter stops after ~2 sec.
 *
 *  'AlwaysSpin' constantly enables the shooter.
 */
public class FuelHandlerTestRobot extends CommandRobotBase
{
    private final FuelHandler fuel_handler = new FuelHandler();

    public FuelHandlerTestRobot()
    {
        RobotOI.joystick.a().onTrue(fuel_handler.toggleIntake());
        RobotOI.joystick.y().onTrue(fuel_handler.shoot());
    }
}
