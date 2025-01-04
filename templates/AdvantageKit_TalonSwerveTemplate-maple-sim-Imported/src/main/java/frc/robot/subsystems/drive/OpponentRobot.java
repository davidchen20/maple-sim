// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OpponentRobot extends SubsystemBase {
   /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
   public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
    new Pose2d(-6, 0, new Rotation2d()),
    new Pose2d(-5, 0, new Rotation2d()),
    new Pose2d(-4, 0, new Rotation2d()),
    new Pose2d(-3, 0, new Rotation2d()),
    new Pose2d(-2, 0, new Rotation2d())
};

private final SimplifiedSwerveDriveSimulation driveSimulation;
private final Pose2d queeningPose;
private final int id;

public OpponentRobot(int id) {
    this.id = id;
    this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
    this.driveSimulation = new SimplifiedSwerveDriveSimulation(new SwerveDriveSimulation(
        DRIVETRAIN_CONFIG, 
        queeningPose
    ));

    SimulatedArena.getInstance().addDriveTrainSimulation(
        driveSimulation.getDriveTrainSimulation()
    );
}

public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
  new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
  new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
  new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
  new Pose2d(1.6, 6, new Rotation2d()),
  new Pose2d(1.6, 4, new Rotation2d())
};

/** Joystick drive command for opponent robots */
private Command joystickDrive(XboxController joystick) {
// Obtain chassis speeds from joystick input
final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
      -joystick.getLeftY() * driveSimulation.maxLinearVelocity().in(MetersPerSecond),
      -joystick.getLeftX() * driveSimulation.maxLinearVelocity().in(MetersPerSecond),
      -joystick.getRightX() * driveSimulation.maxAngularVelocity().in(RadiansPerSecond));

// Obtain driverstation facing for opponent driver station
final Supplier<Rotation2d> opponentDriverStationFacing = () ->
      FieldMirroringUtils.getCurrentAllianceDriverStationFacing().plus(Rotation2d.fromDegrees(180));

return Commands.run(() -> {
      // Calculate field-centric speed from driverstation-centric speed
      final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
              joystickSpeeds.get(),
              FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                      .plus(Rotation2d.fromDegrees(180)));
      // Run the field-centric speed
      driveSimulation.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
      }, this)
      // Before the command starts, reset the robot to a position inside the field
      .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
              FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1])));
              public static final OpponentRobot[] instances = new OpponentRobot[...]; // you can create as many opponent robots as you needs
              public static void startOpponentRobotSimulations() {
                  try {
                      instances[0] = new OpponentRobot(0);
                      instances[0].buildBehaviorChooser(
                              PathPlannerPath.fromPathFile("opponent robot cycle path 0"),
                              Commands.none(),
                              PathPlannerPath.fromPathFile("opponent robot cycle path 0 backwards"),
                              Commands.none(),
                              new XboxController(1));
                  } catch (Exception e) {
                      DriverStation.reportError("Failed to load opponent robot simulation paths, error: " + e.getMessage(), false);
                  }
              }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
