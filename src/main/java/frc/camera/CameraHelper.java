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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.swervelib.SwerveDrivetrain;

/** Helper for using a camera */
public class CameraHelper
{
    private final AprilTagFieldLayout tags;
    private final PhotonCamera camera;
    private final Transform3d robotToCam;
    private final NetworkTableEntry nt_camera, nt_info, nt_distance, nt_tag_period;
    private final Timer tagTimer = new Timer();
    private int successes = 0;
    private double avg_tag_period = 0.02;

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
        nt_info = SmartDashboard.getEntry(camera_name + "Info");
        nt_distance = SmartDashboard.getEntry(camera_name + "Dist");
        nt_tag_period = SmartDashboard.getEntry(camera_name + "TagPeriod");

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

        boolean sawTarget = false;
        for (PhotonPipelineResult result : camera.getAllUnreadResults())
            if (result.hasTargets())
                for (PhotonTrackedTarget target : result.getTargets())
                {
                    sawTarget = true;
                    // Target too far away?
                    double distance = target.bestCameraToTarget.getTranslation().getNorm();
                    nt_distance.setNumber(distance);
                    if (distance > 3.5)
                    {
                        // System.out.println("No best target");
                        continue;
                    }
                    if (target.poseAmbiguity > 0.6)
                        continue;

                    // TODO Vary stddev with distance etc, see
                    // https://www.chiefdelphi.com/t/global-pose-with-ll/513848
                    double fuzzyness = 1.0;
                    // if (distance > 1)
                    //     fuzzyness = distance;
                    // Check drive speed?

                    // Where is that tag on the field?
                    Optional<Pose3d> tag_pose = tags.getTagPose(target.fiducialId);
                    if (tag_pose.isEmpty())
                        continue;

                    Pose3d pose = tag_pose.get();

                    // TESTING...
                    // pose = new Pose3d();

                    // Transform from tag to camera...
                    Transform3d target_to_camera = target.bestCameraToTarget.inverse();
                    String info = String.format("#%02d to cam: %5.2f %5.2f %5.2f  < %5.1f %5.1f %5.1f",
                                                target.fiducialId,
                                                target_to_camera.getX(), target_to_camera.getY(), target_to_camera.getZ(),
                                                Math.toDegrees(target_to_camera.getRotation().getX()),
                                                Math.toDegrees(target_to_camera.getRotation().getY()),
                                                Math.toDegrees(target_to_camera.getRotation().getZ()));
                    nt_info.setString(info);
                    pose = pose.transformBy(target_to_camera);

                    // ... then from camera to center of robot
                    pose = pose.transformBy(robotToCam.inverse());

                    // Map from 3D down to 2D
                    Pose2d position = pose.toPose2d();
                    // System.out.println(target.getFiducialId() + " @ " + tag_pose + " -> " + position);

                    // TODO Filter on valid Z coord
                    // TODO Filter on coords inside field
                    // TODO Filter on heading close to gyro

                    // For tests, force odometry to camera reading
                    // drivetrain.setOdometry(position.getX(), position.getY(), position.getRotation().getDegrees());

                    // For operation, smoothly update location with camera info
                    drivetrain.updateLocationFromCamera(position, result.getTimestampSeconds(), fuzzyness);
                    successes = 50; // 1 second
                }
        nt_camera.setBoolean(successes > 0);
        if (sawTarget)
        {
            double tag_period = tagTimer.get();
            tagTimer.restart();
            // Exponential smoothing, 90:10%
            avg_tag_period = 0.9 * avg_tag_period  +  0.1 * tag_period;
            if (avg_tag_period == 0)
                nt_tag_period.setDouble(0);
            else
                nt_tag_period.setDouble(1.0/avg_tag_period);
        }
    }
}
