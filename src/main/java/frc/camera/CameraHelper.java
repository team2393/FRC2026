// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot;

package frc.camera;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.swervelib.SwerveDrivetrain;

/** Helper for using a camera */
public class CameraHelper
{
    private final AprilTagFieldLayout tags;
    private final PhotonCamera camera;
    private final Transform3d robotToCam;
    private final NetworkTableEntry nt_camera, nt_distance;
    private int successes = 0;

    /** @param tags Field info
     *  @param camera_name Camera name ("Front", "Back") in photonvision network tablee ntries
     *                     Will create tags "FrontCamera" for status, "FrontDist" for distance
     *  @param pos_x Camera pos. relative to center of robot, X
     *  @param pos_y Y
     *  @param pos_z Z
     *  @param heading Camera heading (yaw)
     *  @param pitch Camera down/up tilt
     */
    public CameraHelper(AprilTagFieldLayout tags, String camera_name,
                        double pos_x, double pos_y, double pos_z,
                        double heading,
                        double pitch)
    {
        this.tags = tags;
        camera = new PhotonCamera(camera_name);

        nt_camera = SmartDashboard.getEntry(camera_name + "Camera");
        nt_distance = SmartDashboard.getEntry(camera_name + "Dist");

        // XXX Allow access to the camera from a computer when tethered to the USB port on the roboRIO
        // PortForwarder.add(5800, "photonvision.local", 5800);

        // Where is the camera mounted relative to the center of the robot?
        // Example: mounted facing forward, 30cm forward of center, 10cm up from floor.
        robotToCam = new Transform3d(new Translation3d(pos_x, pos_y, pos_z),
                                     new Rotation3d(0,
                                                    Math.toRadians(pitch),
                                                    Math.toRadians(heading)));
    }

    /** Call periodically to update drivetrain with camera info */
    public void updatePosition(SwerveDrivetrain drivetrain)
    {
        --successes;
        if (successes < 0)
            successes = 0;
        if (! camera.isConnected())
        {
            // System.out.println("Camera " + camera.getName() + " is disconnected!!!");
            nt_camera.setBoolean(successes > 0);
            return;
        }

        for (PhotonPipelineResult result : camera.getAllUnreadResults())
            if (result.hasTargets())
                for (PhotonTrackedTarget target : result.getTargets())
                {
                    // Traget too far away?
                    double distance = target.bestCameraToTarget.getTranslation().getNorm();
                    nt_distance.setNumber(distance);
                    if (distance > 2.5)
                    {
                        // System.out.println("No best target");
                        continue;
                    }

                    // Where is that tag on the field?
                    Optional<Pose3d> tag_pose = tags.getTagPose(target.fiducialId);
                    if (tag_pose.isEmpty())
                        continue;

                    // System.out.println(target.bestCameraToTarget);
                    // Transform from tag to camera, then from camera to center of robot
                    Pose3d pose = tag_pose.get();
                    pose = pose.transformBy(target.bestCameraToTarget.inverse());
                    pose = pose.transformBy(robotToCam.inverse());
                    Pose2d position = pose.toPose2d();
                    // System.out.println(target.getFiducialId() + " @ " + tag_pose + " -> " + position);

                    // TODO Filter on valid Z coord
                    // TODO Filter on coords inside field
                    // TODO Filter on heading close to gyro

                    // For tests, force odometry to camera reading
                    // drivetrain.setOdometry(position.getX(), position.getY(), position.getRotation().getDegrees());

                    // For operation, smoothly update location with camera info
                    drivetrain.updateLocationFromCamera(position, result.getTimestampSeconds());
                    successes = 50; // 1 second
                }
        nt_camera.setBoolean(successes > 0);
    }
}
