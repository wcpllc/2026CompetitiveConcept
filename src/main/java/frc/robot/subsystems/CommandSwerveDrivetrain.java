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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Translation2d;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private static final double kBumpTiltThresholdDegrees = 5.0;
    private static final double kOdometryUpdateFrequencyHz = 100.0;
    private static final double kTelemetryUpdateFrequencyHz = 20.0;
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
    private final List<StatusSignal<Current>> driveSupplyCurrentSignals;
    private final List<StatusSignal<Current>> steerSupplyCurrentSignals;
    private final List<StatusSignal<Temperature>> driveTempSignals;
    private final List<StatusSignal<Temperature>> steerTempSignals;
    private final double[] driveCurrents = new double[4];
    private final double[] steerCurrents = new double[4];
    private final double[] driveTemps = new double[4];
    private final double[] steerTemps = new double[4];

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, kOdometryUpdateFrequencyHz, modules);

        pitchSignal = getPigeon2().getPitch();
        rollSignal = getPigeon2().getRoll();
        driveSupplyCurrentSignals = createDriveSupplyCurrentSignals();
        steerSupplyCurrentSignals = createSteerSupplyCurrentSignals();
        driveTempSignals = createDriveTempSignals();
        steerTempSignals = createSteerTempSignals();

        BaseStatusSignal.setUpdateFrequencyForAll(kTelemetryUpdateFrequencyHz, pitchSignal, rollSignal);
        for (int i = 0; i < driveSupplyCurrentSignals.size(); i++) {
            driveSupplyCurrentSignals.get(i).setUpdateFrequency(kTelemetryUpdateFrequencyHz);
            steerSupplyCurrentSignals.get(i).setUpdateFrequency(kTelemetryUpdateFrequencyHz);
            driveTempSignals.get(i).setUpdateFrequency(kTelemetryUpdateFrequencyHz);
            steerTempSignals.get(i).setUpdateFrequency(kTelemetryUpdateFrequencyHz);
        }
        BaseStatusSignal.refreshAll(pitchSignal, rollSignal);

        configureAutoBuilder();
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

    private List<StatusSignal<Current>> createDriveSupplyCurrentSignals() {
        var modules = getModules();
        List<StatusSignal<Current>> signals = new java.util.ArrayList<>(modules.length);
        for (int i = 0; i < modules.length; i++) {
            signals.add(modules[i].getDriveMotor().getSupplyCurrent());
        }
        return signals;
    }

    private List<StatusSignal<Current>> createSteerSupplyCurrentSignals() {
        var modules = getModules();
        List<StatusSignal<Current>> signals = new java.util.ArrayList<>(modules.length);
        for (int i = 0; i < modules.length; i++) {
            signals.add(modules[i].getSteerMotor().getSupplyCurrent());
        }
        return signals;
    }

    private List<StatusSignal<Temperature>> createDriveTempSignals() {
        var modules = getModules();
        List<StatusSignal<Temperature>> signals = new java.util.ArrayList<>(modules.length);
        for (int i = 0; i < modules.length; i++) {
            signals.add(modules[i].getDriveMotor().getDeviceTemp());
        }
        return signals;
    }

    private List<StatusSignal<Temperature>> createSteerTempSignals() {
        var modules = getModules();
        List<StatusSignal<Temperature>> signals = new java.util.ArrayList<>(modules.length);
        for (int i = 0; i < modules.length; i++) {
            signals.add(modules[i].getSteerMotor().getDeviceTemp());
        }
        return signals;
    }

    private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the
          // case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
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
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
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
            // MegaTag2 requires the robot's gyro orientation to compute pose
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
                
                if (omega < 3.0 || est.tagCount > 1) {
                   //  if (!vision.hasHighSingleTagAmbiguity(est)) {
                         addVisionMeasurement(est.pose, est.timestampSeconds, stdDevs);
                  //  }
                }
             }

            Logger.recordOutput("Vision/Poses", visionPoses);
            Logger.recordOutput("Vision/StdDevsXY", visionStdDevsXY);
            Logger.recordOutput("Vision/StdDevsTheta", visionStdDevsTheta);
            Logger.recordOutput("Vision/TagCounts", visionTagCounts);
            Logger.recordOutput("Vision/AvgTagDists", visionAvgTagDists);
            Logger.recordOutput("Vision/Latencies", visionLatencies);

            //Log heading disagreement between vision and odometry for quick diagnosis
            // if (!estimates.isEmpty()) {
            //     Pose2d bestVisionPose = estimates.get(0).pose;
            //     Pose2d odomPose = getState().Pose;
            //     double headingErrorDeg = bestVisionPose.getRotation().minus(odomPose.getRotation()).getDegrees();
            //     double translationErrorM = bestVisionPose.getTranslation().getDistance(odomPose.getTranslation());
            //     Logger.recordOutput("Vision/HeadingErrorDeg", headingErrorDeg);
            //     Logger.recordOutput("Vision/TranslationErrorM", translationErrorM);
            // }
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

        // for (int i = 0; i < driveSupplyCurrentSignals.size(); i++) {
        //     driveCurrents[i] = driveSupplyCurrentSignals.get(i).getValueAsDouble();
        //     steerCurrents[i] = steerSupplyCurrentSignals.get(i).getValueAsDouble();
        //     driveTemps[i] = driveTempSignals.get(i).getValueAsDouble();
        //     steerTemps[i] = steerTempSignals.get(i).getValueAsDouble();
        // }

        // Logger.recordOutput("Drive/DriveSupplyCurrents", driveCurrents);
        // Logger.recordOutput("Drive/SteerSupplyCurrents", steerCurrents);
        // Logger.recordOutput("Drive/DriveTemperatures", driveTemps);
        // Logger.recordOutput("Drive/SteerTemperatures", steerTemps);
        
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
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
    m_simNotifier.startPeriodic(kSimLoopPeriod);
    
    // Initialize simulation pose to inside the field on black line for blue alliance
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
        // Push one sim update instead of blocking the main loop for 0.1s
        mapleSimSwerveDrivetrain.update();
        }
        super.resetPose(pose);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]t, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}
