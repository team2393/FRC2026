Team 2393 FRC Season 2026
=========================

![logo](logo.jpg)

Software manual: https://docs.wpilib.org/en/latest/

 * The "Introduction" has links to Java learning resources.
   See also [Free Java Book](https://greenteapress.com/wp/think-java-2e/)
 * See "Step 2: Installing Software", "WPILib Installation Guide".
 * See "Basic Programming", "Git Version Control" for installing `git`

Game manual: https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system

Other links:

 * Forum -https://www.chiefdelphi.com/
 * Mechanical resources - https://www.frcdesign.org/

Get robot code
--------------
 * Start "WPILib VS Code". Invoke "View", "Command Palette...", "Git clone".
   Enter the URL `https://github.com/team2393/FRC2026.git`.
   For a target location, create a folder "git" in your home directory
   and place the git clone there. Select "Open" when then asked to open
   what you just fetched from git.
 * "View", "Command Palette", "WPILib: Simulate Robot Code".
   After a short while, a "Pick extension to run" prompt will appear.
   Select "Sim GUI", press "OK".

![servebot](swervebot.jpg)

Timeline
--------

 * January 10: Kickoff
   - Hardin Valley Academy,11345 Hardin Valley Rd., Knoxville
   - https://www.firstinspires.org/event-detail?eventId=76547

 * Use chassis, roboRIO, power panel, radio from last year's robot
 * Setup roboRIO with current firmware
 * Setup own laptops with current tools
 * Configure radio for 2.4 GHz usage with laptop:
   - Connect via DS port and see http://radio.local or http://192.168.69.1
   - Upgrade firmware https://frc-radio.vivid-hosting.net/overview/upgrading-firmware
   - Configure Robot Radio mode, team number, SSID and 2.4 GHz password, enable 2.4 GHz wifi via DIP #3
   - Check other DIPs to disable power-over-ethernet which could destroy vision processors
   - For access point, set AP and robot radio to the same team and SSID
   - After competition, connect via network cable, open http://10.23.93.1, re-enable 2.4 GHz and set password
 * Tune swerve drivetrain
 * Find camera (limelight 2, maybe later Pi)
 * Jan. 16: Use camera to rotate on target and estimate distance (2D, RotateToTarget).
   Works OK as long as stream resolution is small.
 * Jan. 17: Calibrate camera, use to locate robot on field.
   Mostly works, but X distance off by 10 or 20 cm.
   Mean error 0.27px, FOV 71.2, 56.0, 83.0.
   Calibrated again, and X distance appears good to 1..2 cm!
   Mean error 0.34px, FOV 62.57, 48.92, 74.40.
 * Rotate onto alliance hub using that information
 * Find team/drive laptop:
   It has all the tools, but wouldn't connect to radio,
   fixed with radio update
   https://www.chiefdelphi.com/t/vh109-radio-wifi-doesnt-connect/513308/3
 * Jan 31: Need to re-assemble drive base
 * Feb 23: Need to re-connect drive base wiring
 * Feb 26: Start to test feeder, feeder sensor, spinner
 * Mar 2: Test intake arm, storage, feeder, shooter,
          configure AimToHub table of hood and speed settings.
          Intake mover disabled until fixed.
 * Test/tune combined game piece handling
 * Mar 7: On practice field, test cameras, relative vs. abs. drive,
          auto (back, shoot, shoot & go to trench for blue side)
 * Tune spinner spood
 * Combine aming and shooting
 * Auto ideas

 * March 18 - 21: Smoky Mountains Regional
   - Sevierville Convention Center, 202 Gists Creek Rd, Sevierville
   - https://www.firstinspires.org/event-detail?eventId=76272

 * April 8 - 11: Rocket City Regional
   - Von Braun Center, 700 Monroe Street SW, Huntsville, AL USA
   - https://www.firstinspires.org/event-detail?eventId=76307
   - https://frc-events.firstinspires.org/2026/alhu

Network
-------

| Address     | Device                 | Protocoll                     |
| ----------- | ---------------------- | ------------------------------|
| 10.23.93.1  | Robot radio            | http                          |
| 10.23.93.2  | RoboRIO                | http, ssh 'admin' or 'lvuser' |
| 10.23.93.4  | "Gateway", AP radio    | http                          |
| 10.23.93.11 | Default for new camera | http                          |
| 10.23.93.12 | "Front" camera         | http                          |
| 10.23.93.13 | "Back" camera          | http                          |
| 10.23.93.14 | "Front1" camera        | http://10.23.93.14:5800       |
| 10.23.93.15 | "Front2" camera        | http                          |
| 10.23.93.?? | Drive station          | ssh to robot, Network Tables  |

See https://docs.wpilib.org/en/latest/docs/networking/networking-introduction/ip-configurations.html

Camera (Photon Vision, Pi)
--------------------------

See https://docs.photonvision.org/en/latest/docs/quick-start/index.html
and https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-4.

For LL4, power camera on while button is pressed.
Balena etcher failed to detect "compute module".
Limelight Hardware Manager was able to detect the camera,
but needs photonvision-limelight4.img.xz to first be expanded into *.img.

 * Get `photonvision-...-linuxarm64_RaspberryPi.img.xz`
   from https://github.com/PhotonVision/photonvision/releases
 * Use etcher or raspberry tool to write memory card
 * Connect Pi to robot network
 * Access from laptop as http://photonvision.local:5800 (may take several attempts!)
 * Under "Settings", set Team Number to 2393
 * Change IP Assignment Mode from DHCP to Static,
   set address to `10.23.93.14` and camera name to `Front1` as per table.
   (Static `10.TE.AM.6-19` are general purpose.
    `10.TE.AM.11` is also often used for camera, leave that for Limelight)
 * Leave "Hostname" as `photonvision` for Front,
   change to `photonvision2` for Back camera
 * Press SAVE, restart Pi

From now on, access photonvision via http://10.23.93.12:5800 !

 * Setup a camera for first test:
   * "activate" detected USB camera
   * Select "Driver mode"
   * Find settings that "work"  (disable auto-exposure, select )

 * Create "Tag"pipeline for April Tags.
   Should recognize tags in 2D

 * Calibrate camera, test accuracy of detected targets


From https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags

 * For ideal tracking, consider the following:
    - Your tags should be as flat as possible.
    - Your Limelight should be mounted above or below tag height and angled up/down such that the target is centered.
    - Your target should look as trapezoidal as possible from your camera's perspective.
      You don't want your camera to ever be completely "head-on" with a tag if you want to avoid tag flipping.

 * There is an interplay between the following variables for AprilTag Tracking:
    - Increasing capture resolution will always increase 3D accuracy and increase 3d stability.
      This will also reduce the rate of ambiguity flipping from most perspectives.
      It will usually increase range. This will reduce pipeline framerate.
    - Increasing detector downscale will always increase pipeline framerate.
      It will decrease effective range, but in some cases this may be negligible.
      It will not affect 3D accuracy, 3D stability, or decoding accuracy.
    - Reducing exposure will always improve motion-blur resilience. This is actually really easy to observe. This may reduce range.
    - Reducing the brightness and contrast of the image will generally improve pipeline framerate and reduce range.
    - Increasing Sensor gain allows you to increase brightness without increasing exposure. It may reduce 3D stability, and it may reduce tracking stability.

Profiling
---------

'VisualVM', available from https://visualvm.github.io,
allows you to see how much CPU and memory the code is using on the RoboRIO,
and where it spends its time.

Assuming you unpacked it to \Users\Public\wpilib,
start it from a command prompt to pass the JDK location like this:

```
cd \Users\Public\wpilib\visualvm\bin
visualvm --jdkhome \Users\Public\wpilib\2026\jdk
```

In `build.gradle`, this addition to the FRCJavaArtifact section
allows VisualVM to access the JVM running on the robot:

```
frcJava(getArtifactTypeClass('FRCJavaArtifact')) {
    // Enable VisualVM connection
    jvmArgs.add("-Dcom.sun.management.jmxremote=true")
    jvmArgs.add("-Dcom.sun.management.jmxremote.port=1198")
    jvmArgs.add("-Dcom.sun.management.jmxremote.local.only=false")
    jvmArgs.add("-Dcom.sun.management.jmxremote.ssl=false")
    jvmArgs.add("-Dcom.sun.management.jmxremote.authenticate=false")
    jvmArgs.add("-Djava.rmi.server.hostname=10.23.93.2")
}
```

To connect to the program running on the robot:
 * File, Add JMX Connection
 * 'Connection:' 172.22.11.2:1198 respectively 10.23.93.2:1198
 * Check 'Do not require SSL connection'
 * A new entry with a 'pid' should appear under the 'Remote' list.
   Double-click, then check 'Monitor', 'Sample.. CPU' etc.
