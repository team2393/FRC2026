// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervebot;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Simple PhotonVision camera helper for 2D targets
 * 
 *  Directly reads network table entries, names need to match camera settings
 */
public class Camera2D
{
    private final BooleanSubscriber has_target;
    private final DoubleSubscriber target_yaw;

    public Camera2D()
    {
        SmartDashboard.getEntry("photonvision");
        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision");
        NetworkTable camera = table.getSubTable("HD_Pro_Webcam_C920");
        has_target = camera.getBooleanTopic("hasTarget").subscribe(false);
        target_yaw = camera.getDoubleTopic("targetYaw").subscribe(0);
        // TODO Also track target size to get distance estimate
    }

    /** @return Degrees to target or NAN */
    public double getAngleToTarget()
    {
        if (has_target.get())
            return target_yaw.get();
        return Double.NaN;
    }
}
