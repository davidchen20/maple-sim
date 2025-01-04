package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Telemetry;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.*;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class MapleSimSwerve implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation simulatedDrive;

    public MapleSimSwerve() {
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(new SwerveModuleSimulationConfig(
                        // adjust motor type as needed
                        DCMotor.getFalcon500(1),
                        DCMotor.getFalcon500(1),
                        Constants.Swerve.driveGearRatio,
                        Constants.Swerve.angleGearRatio,
                        Volts.of(0.1),
                        Volts.of(0.1),
                        Meters.of(Constants.Swerve.chosenModule.wheelDiameter / 2),
                        KilogramSquareMeters.of(0.02),
                        1.2));

        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(0, 0, new Rotation2d())));

        this.simulatedDrive.withCurrentLimits(Amps.of(60), Amps.of(20));

        // register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.simulatedDrive.runChassisSpeeds(
                new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
                new Translation2d(),
                fieldRelative,
                true);
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return simulatedDrive.getLatestModulePositions();
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return simulatedDrive.getMeasuredStates();
    }

    @Override
    public ChassisSpeeds getMeasuredSpeeds() {
        return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
    }

    @Override
    public Rotation2d getGyroYaw() {
        return simulatedDrive.getRawGyroAngle();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        simulatedDrive.periodic();

        Telemetry.getInstance().feedSimulationRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        Telemetry.getInstance().feedOdometryPose(getPose());
    }

    public AbstractDriveTrainSimulation getSimDrive() {
        return simulatedDrive.getDriveTrainSimulation();
    }
}
