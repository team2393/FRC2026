// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

/** What on the robot is connected where and how? */
public class RobotMap
{
    // Constants define CAN IDs or RoboRIO ports
    // For CAN ID, number should match the power distribution panel port

    // RoboRIO: 10 Amp, port 21
    // Radio  : 10 Amp, port 20

    // Kraken drive motors: 40 Amp Fuse
    public static final int FRONT_LEFT_DRIVE  =  3;
    public static final int FRONT_RIGHT_DRIVE = 19;
    public static final int BACK_RIGHT_DRIVE  = 16;
    public static final int BACK_LEFT_DRIVE   =  5;

    // Kraken rotator motors: 40 Amp Fuse
    public static final int FRONT_LEFT_ROTATE  =  1;
    public static final int FRONT_RIGHT_ROTATE = 18;
    public static final int BACK_RIGHT_ROTATE  = 17;
    public static final int BACK_LEFT_ROTATE   =  6;

    public static final int HOOD               = 10;

    // CANCoder angle sensors: Chained to channel 0 on PDP with 40 Amp Fuse,
    // then 10 Amp fuse on mini power panel
    public static final int FRONT_LEFT_ANGLE  = 1;
    public static final int FRONT_RIGHT_ANGLE = 57;
    public static final int BACK_RIGHT_ANGLE  = 4;
    public static final int BACK_LEFT_ANGLE   = 24;

    // Falcon CAN IDs
    public static final int INTAKE_MOVER = 18;
    public static final int INTAKE_ARM = 19;
    public static final int STOREAGE_MOVER = 20;
    public static final int SPINNER = 20;
    public static final int SPINNER2 = 21;

    // DIO channels
    public static final int HOOD_HOME = 2;
    public static final int INTAKE_OPENER = 4;
    public static final int STORAGE_SENSOR = 5;
}
