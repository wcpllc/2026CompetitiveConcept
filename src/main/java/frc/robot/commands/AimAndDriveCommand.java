package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);
    private static final double kDebugPrintIntervalSeconds = 0.5;
    private static final double kMinAimVectorNormMeters = 1e-6;

    private final CommandSwerveDrivetrain swerve;
    private final DriveInputSmoother inputSmoother;
    private static final double kPoseEdgeMarginMeters = 0.1;
    private boolean poseWarningIssued = false;
    private double lastDebugPrintTimestamp = 0.0;

    private static final Translation2d kDefaultBlueHubPosition = new Translation2d(Inches.of(182.105), Inches.of(158.845));
    private static final Translation2d kDefaultRedHubPosition = new Translation2d(Inches.of(469.115), Inches.of(158.845));

    public static final AprilTagFieldLayout layout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double fieldLength = layout.getFieldLength();
    public static final double fieldWidth = layout.getFieldWidth();

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand(
        CommandSwerveDrivetrain swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(CommandSwerveDrivetrain swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        if (!currentPoseIsValid()) {
            return false;
        }
        final Rotation2d targetHeading = getTargetHeadingInOperatorPerspective();
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.minus(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getTargetHeadingInOperatorPerspective() {
        return getTargetHeadingInFieldFrame().minus(swerve.getOperatorForwardDirection());
    }

    private Rotation2d getTargetHeadingInFieldFrame() {
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Translation2d hubPosition = DriverStation.getAlliance().get().equals(Alliance.Blue) ? kDefaultBlueHubPosition : kDefaultRedHubPosition;;
        return hubPosition.minus(robotPosition).getAngle();
    }

    private boolean isPoseValid(Pose2d pose) {
        if (pose == null) {
            return false;
        }
        final Translation2d translation = pose.getTranslation();
        final double x = translation.getX();
        final double y = translation.getY();
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return false;
        }
        return x >= -kPoseEdgeMarginMeters
            && x <= fieldLength + kPoseEdgeMarginMeters
            && y >= -kPoseEdgeMarginMeters
            && y <= fieldWidth + kPoseEdgeMarginMeters;
    }

    private boolean currentPoseIsValid() {
        return isPoseValid(swerve.getState().Pose);
    }

    @Override
    public void initialize() {
         poseWarningIssued = false;
         ensurePoseValidWithWarning();
    }

    @Override
    public void execute() {
        if (!ensurePoseValidWithWarning()) {
             swerve.requestIdle();
             return;
         }

        //DEBUG: Print aim diagnostics
        final double now = Timer.getFPGATimestamp();
        if (now - lastDebugPrintTimestamp >= kDebugPrintIntervalSeconds) {
            lastDebugPrintTimestamp = now;
            final Pose2d pose = swerve.getState().Pose;
            final Translation2d hubPosition = Landmarks.hubPosition();
            final Rotation2d currentHeading = pose.getRotation();
            final Rotation2d targetFieldHeading = getTargetHeadingInFieldFrame();
            final Rotation2d targetOpHeading = getTargetHeadingInOperatorPerspective();
            final Rotation2d operatorForward = swerve.getOperatorForwardDirection();
            final double degreesToTurn = targetOpHeading.getDegrees()
                - currentHeading.minus(operatorForward).getDegrees();
            final String allianceStr = DriverStation.getAlliance()
                .map(a -> a.toString())
                .orElse("EMPTY (defaulting to Blue!)");
            System.out.printf(
                "[AimDebug] Alliance=%s | OpForward=%.1f° | Robot=(%.2f, %.2f) heading=%.1f° | Hub=(%.2f, %.2f) | Target(field)=%.1f° Target(op)=%.1f° | Turn=%.1f°%n",
                allianceStr, operatorForward.getDegrees(),
                pose.getX(), pose.getY(), currentHeading.getDegrees(),
                hubPosition.getX(), hubPosition.getY(),
                targetFieldHeading.getDegrees(), targetOpHeading.getDegrees(),
                degreesToTurn
            );
            Logger.recordOutput("Aim/Alliance", allianceStr);
            Logger.recordOutput("Aim/OpForwardDeg", operatorForward.getDegrees());
            Logger.recordOutput("Aim/HubPosition", new Pose2d(hubPosition, new Rotation2d()));
            Logger.recordOutput("Aim/TargetFieldHeadingDeg", targetFieldHeading.getDegrees());
            Logger.recordOutput("Aim/TargetOpHeadingDeg", targetOpHeading.getDegrees());
            Logger.recordOutput("Aim/DegreesToTurn", degreesToTurn);
        }

        // If you want to use the above for debugging without the robot moving, comment out below
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        final Rotation2d targetHeading = getTargetHeadingInOperatorPerspective();
        swerve.setControl(
                fieldCentricFacingAngleRequest
                 .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                 .withVelocityY(Driving.kMaxSpeed.times(input.left))
                 .withTargetDirection(targetHeading)
        );
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean ensurePoseValidWithWarning() {
        final boolean valid = currentPoseIsValid();
        if (!valid) {
            if (!poseWarningIssued) {
                DriverStation.reportWarning("Auto aim blocked: robot pose outside field bounds", false);
                poseWarningIssued = true;
            }
        } else {
            poseWarningIssued = false;
        }
        return valid;
    }
}
