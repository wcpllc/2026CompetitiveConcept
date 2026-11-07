package frc.robot.subsystems;

import java.util.List;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

// BLine-Lib Imports
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;
import frc.robot.utils.simulation.SimSwerveConstants;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private static final double kBumpTiltThresholdDegrees = 5.0;
    private static final double kOdometryUpdateFrequencyHz = 100.0;
    private static final double kTelemetryUpdateFrequencyHz = 20.0;
    /**
     * MegaTag2 solves position from the gyro heading we send it, so a good estimate comes back facing
     * the way the gyro says. One that does not is a bad solve (mttd q8 auto: the Limelight returned
     * field center facing 0 degrees for 20 frames and pulled the pose 5 m off).
     */
    private static final double kMaxVisionHeadingErrorDegrees = 15.0;
    private Notifier m_simNotifier = null;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private NeutralModeValue currentDriveNeutralMode = null;
    private NeutralModeValue currentSteerNeutralMode = null;
    private final SwerveRequest.SwerveDriveBrake lockWheelsRequest = new SwerveRequest.SwerveDriveBrake();

    private VisionSubsystem vision;
    //uses this to be able to lock angle of drivetrain a certain way
    private final SwerveRequest.FieldCentricFacingAngle faceAngleRequest =
    new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(0.02)
        .withRotationalDeadband(0.01);

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<Angle> rollSignal;
    /** Per-module currents, temperatures, encoder health, and faults. See docs/drivetrain-log-guide.md. */
    private final SwerveModuleDiagnostics moduleDiagnostics;
    /** Team settings layered over the generated TunerConstants, plus a check that they reached the motors. */
    private final SwerveConfigOverrides configOverrides;

    // BLine Path Follower Builder
    private final FollowPath.Builder pathBuilder;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, kOdometryUpdateFrequencyHz, SwerveConfigOverrides.apply(modules));

        pitchSignal = getPigeon2().getPitch();
        rollSignal = getPigeon2().getRoll();
        moduleDiagnostics = new SwerveModuleDiagnostics(getModules());
        configOverrides = new SwerveConfigOverrides(getModules());

        BaseStatusSignal.setUpdateFrequencyForAll(kTelemetryUpdateFrequencyHz, pitchSignal, rollSignal);
        BaseStatusSignal.refreshAll(pitchSignal, rollSignal);

        // Initialize BLine FollowPath Builder
        pathBuilder = new FollowPath.Builder(
            this,                                                   // Subsystem requirement
            this::getPose,                                          // Field-relative Pose2d supplier
            () -> getState().Speeds,                                // Robot-relative ChassisSpeeds supplier
            speeds -> setControl(pathApplyRobotSpeeds.withSpeeds(speeds)), // Robot-relative speed consumer
            new PIDController(10.0, 0.0, 0.0),                      // Translation PID
            new PIDController(7.0, 0.0, 0.0),                       // Rotation PID
            new PIDController(2.0, 0.0, 0.0)                        // Cross-track PID
        ).withDefaultShouldFlip()
         .withTRatioBasedTranslationHandoffs(true);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void setSteerNeutralMode(NeutralModeValue neutralMode) {
        if (neutralMode == currentSteerNeutralMode) {
            return;
        }
        for (var module : getModules()) {
            module.getSteerMotor().setNeutralMode(neutralMode);
        }
        currentSteerNeutralMode = neutralMode;
    }

    public void setDriveNeutralMode(NeutralModeValue neutralMode) {
        if (neutralMode == currentDriveNeutralMode) {
            return;
        }
        for (var module : getModules()) {
            module.getDriveMotor().setNeutralMode(neutralMode);
        }
        currentDriveNeutralMode = neutralMode;
    }

    public void requestIdle() {
        setControl(idleRequest);
    }

    /**
     * Builds and returns a BLine path-following command for the given path file in deploy/autos/paths/
     * 
     * @param pathName The name of the path file (without .json extension)
     * @return Command to follow the path
     */
    public Command followPath(String pathName) {
        Path path = new Path(pathName);
        Command cmd = pathBuilder
            .withPoseReset(this::resetPose)
            .build(path);
        // Clear pose reset so subsequent path segments in a multi-path routine don't reset pose again
        pathBuilder.withPoseReset(ignored -> {});
        return cmd;
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    //for shooting on the move
    public Pose2d getPose() {
        return getState().Pose;
      }
  
      public void setVision(VisionSubsystem vision) {
        this.vision = vision;
      }
  
      //now we can set the angle that we move at both when we are moving and when we are rotating
      public void driveFacingAngle(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        Rotation2d targetAngle
        ) {
          setControl(
            faceAngleRequest
              .withVelocityX(vxMetersPerSecond)
              .withVelocityY(vyMetersPerSecond)
              .withTargetDirection(targetAngle)
          );
      }
  
      //if robot isnt moving wed use this instead
      public void lockHeading(Rotation2d targetAngle) {
        driveFacingAngle(0.0, 0.0, targetAngle);
      }
    
    public void zeroHeading() {
        Rotation2d forward = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? kRedAlliancePerspectiveRotation
            : kBlueAlliancePerspectiveRotation;
        super.resetRotation(forward);
        seedFieldCentric();
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                if (!m_hasAppliedOperatorPerspective) {
                    seedFieldCentric();
                }
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SwerveDriveState state = getState();
        if (vision != null) {
            double omega = Math.abs(state.Speeds.omegaRadiansPerSecond);
            double gyroYawDegrees = state.Pose.getRotation().getDegrees();
            LimelightHelpers.SetRobotOrientation("limelight-shooter", gyroYawDegrees, 0, 0, 0, 0, 0);
            List<LimelightHelpers.PoseEstimate> estimates = vision.getAllPoseEstimates();

            Logger.recordOutput("Vision/OmegaRadPerSec", omega);
            Logger.recordOutput("Vision/GyroYawDegrees", gyroYawDegrees);
            Logger.recordOutput("Vision/EstimateCount", estimates.size());

            Pose2d[] visionPoses = new Pose2d[estimates.size()];
            double[] visionStdDevsXY = new double[estimates.size()];
            double[] visionStdDevsTheta = new double[estimates.size()];
            double[] visionTagCounts = new double[estimates.size()];
            double[] visionAvgTagDists = new double[estimates.size()];
            double[] visionLatencies = new double[estimates.size()];

             for (int i = 0; i < estimates.size(); i++) {
                 LimelightHelpers.PoseEstimate est = estimates.get(i);
                 Matrix<N3, N1> stdDevs = vision.getVisionStdDevsForEstimate(est);

                visionPoses[i] = est.pose;
                visionStdDevsXY[i] = stdDevs.get(0, 0);
                visionStdDevsTheta[i] = stdDevs.get(2, 0);
                visionTagCounts[i] = est.tagCount;
                visionAvgTagDists[i] = est.avgTagDist;
                visionLatencies[i] = est.latency;
                
double headingErrorDegrees = Math.abs(est.pose.getRotation().minus(state.Pose.getRotation()).getDegrees());
                if ((omega < 3.0 || est.tagCount > 1) && headingErrorDegrees < kMaxVisionHeadingErrorDegrees) {
                    addVisionMeasurement(est.pose, est.timestampSeconds, stdDevs);
                }
             }

            Logger.recordOutput("Vision/Poses", visionPoses);
            Logger.recordOutput("Vision/StdDevsXY", visionStdDevsXY);
            Logger.recordOutput("Vision/StdDevsTheta", visionStdDevsTheta);
            Logger.recordOutput("Vision/TagCounts", visionTagCounts);
            Logger.recordOutput("Vision/AvgTagDists", visionAvgTagDists);
            Logger.recordOutput("Vision/Latencies", visionLatencies);
        }

        if (mapleSimSwerveDrivetrain != null) {
            Pose2d simPose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
            super.resetPose(simPose);
            Logger.recordOutput("Drive/Pose", simPose);
        } else {
            Logger.recordOutput("Drive/Pose", state.Pose);
        }

        Logger.recordOutput("Drive/Pose3d", getPose3d());
        Logger.recordOutput("Drive/X", state.Pose.getX());
        Logger.recordOutput("Drive/Y", state.Pose.getY());
        Logger.recordOutput("Drive/Rotation3d", getRobotRotation3d());
        Logger.recordOutput("Drive/PitchDegrees", getPitchDegrees());
        Logger.recordOutput("Drive/RollDegrees", getRollDegrees());
        Logger.recordOutput("Drive/TiltMagnitudeDegrees", getTiltMagnitudeDegrees());
        Logger.recordOutput("Drive/OnBump", isRobotOnBump());

        moduleDiagnostics.logPeriodic();
        if (DriverStation.isDisabled()) {
            configOverrides.verifyPeriodic();
        }
        
        Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());
        Logger.recordOutput("Drive/TargetStates", getState().ModuleTargets);
        Logger.recordOutput("Drive/MeasuredStates", getState().ModuleStates);
        Logger.recordOutput("Drive/MeasuredSpeeds", getState().Speeds);
    }

    public void logStickyFaults() {
        var swerveModules = getModules();
        for (int i = 0; i < swerveModules.length; i++) {
            int driveId = swerveModules[i].getDriveMotor().getDeviceID();
            int steerId = swerveModules[i].getSteerMotor().getDeviceID();
            Logger.recordOutput("Drive/Faults/Drive" + driveId, swerveModules[i].getDriveMotor().getStickyFaultField().getValueAsDouble());
            Logger.recordOutput("Drive/Faults/Steer" + steerId, swerveModules[i].getSteerMotor().getStickyFaultField().getValueAsDouble());
        }
    }

    public double getPitchDegrees() {
        return pitchSignal.getValue().in(Degrees);
    }

    public double getRollDegrees() {
        return rollSignal.getValue().in(Degrees);
    }

    public double getTiltMagnitudeDegrees() {
        return Math.hypot(getPitchDegrees(), getRollDegrees());
    }

    public Rotation3d getRobotRotation3d() {
        Rotation3d pigeonRotation = getPigeon2().getRotation3d();
        double yawRadians = getState().Pose.getRotation().getRadians();
        return new Rotation3d(pigeonRotation.getX(), pigeonRotation.getY(), yawRadians);
    }

    public Pose3d getPose3d() {
        Pose2d pose2d = getState().Pose;
        return new Pose3d(pose2d.getX(), pose2d.getY(), 0.0, getRobotRotation3d());
    }

    public boolean isRobotOnBump() {
        return getTiltMagnitudeDegrees() >= kBumpTiltThresholdDegrees;
    }

    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    private void startSimThread() {
        mapleSimSwerveDrivetrain =
        new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            SimSwerveConstants.ROBOT_MASS,
            SimSwerveConstants.BUMPER_LENGTH_X,
            SimSwerveConstants.BUMPER_LENGTH_Y,
            SimSwerveConstants.DRIVE_MOTOR_WHEEL,
            SimSwerveConstants.STEER_MOTOR_WHEEL,
            SimSwerveConstants.WHEEL_COF,
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);

        m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        m_simNotifier.startPeriodic(kSimLoopPeriod);
        
        double blueAllianceInitialSimX = 3.586;
        double blueAllianceInitialSimY = 5.779;

        Pose2d initialSimPose = new Pose2d(blueAllianceInitialSimX, blueAllianceInitialSimY, new Rotation2d(3.121));
        mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(initialSimPose);
        super.resetPose(initialSimPose);
    }

    public Command lockWheelsCommand() {
        return run(() -> setControl(lockWheelsRequest));
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            mapleSimSwerveDrivetrain.update();
        }
        super.resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}
