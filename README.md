Team 2393 FRC Season 2026
=========================

![logo](logo.jpg)

Software manual: https://docs.wpilib.org/en/latest/

 * The "Introduction" has links to Java learning resources.
   See also [Free Java Book](https://greenteapress.com/wp/think-java-2e/)
 * See "Step 2: Installing Software", "WPILib Installation Guide".
 * See "Basic Programming", "Git Version Control" for installing `git`

Game manual: https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system

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

 * Setup roboRIO with current firmware
 * Setup laptop with current tools
 * Program drivetrain
 * Find camera (limelight? Pi?)
 * Use camera to rotate on target and estimate distance (2D)
 * Calibrate camera, use to locate robot on field,
   then rotate onto nearest target using that information
 * Gamepiece handling

 * March 18 - 21: Smoky Mountains Regional
   - Sevierville Convention Center, 202 Gists Creek Rd, Sevierville
   - https://www.firstinspires.org/event-detail?eventId=76272

 * April 8 - 11: Rocket City Regional
   - Von Braun Center, 700 Monroe Street SW, Huntsville, AL USA
   - https://www.firstinspires.org/event-detail?eventId=76307
   - https://frc-events.firstinspires.org/2026/alhu


Camera (Photon Vision, Pi)
--------------------------

See https://docs.photonvision.org/en/latest/docs/quick-start/index.html

 * Get `photonvision-...-linuxarm64_RaspberryPi.img.xz`
   from https://github.com/PhotonVision/photonvision/releases
 * Use etcher or raspberry tool to write memory card
 * Connect Pi to robot network
 * Access from laptop as http://photonvision.local:5800
 * Under "Settings", set Team Number to 2393
 * Change IP Assignment Mode from DHCP to Static, set address to `10.23.93.12`.
   (Static `10.TE.AM.6-19` are general purpose.
    `10.TE.AM.11` is also often used for camera, leave that for Limelight)
 * Leave "Hostname" as `photonvision`
 * Press SAVE, restart Pi

From now on, access photonvision via http://10.23.93.12:5800

 * Setup a camera for first test:
   * "activate" detected USB camera
   * Select "Driver mode"
   * Find settings that "work"  (disable auto-exposure, select )

 * Create "Tag"pipeline for April Tags.
   Should recognize tags in 2D