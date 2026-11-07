package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Team 9584's swerve settings that differ from what Tuner X generates.
 *
 * <p>{@code TunerConstants.java} is a generated file. When the drivetrain is recalibrated in Tuner X
 * the whole file is replaced, so nothing hand-edited inside it survives. Everything the team changed
 * on purpose lives here instead and is layered on top of the generated constants when the drivetrain
 * is built (see {@link #apply}). That makes it safe to drop in a freshly generated file.
 *
 * <p>After boot, {@link #verifyPeriodic()} reads the settings back from each motor and raises a red
 * dashboard alert if any motor is not running these values. That catches a bad deploy, a Tuner X
 * session that rewrote the motors, or a config apply that failed, before the robot leaves the pit.
 *
 * <p>Why each override exists:
 * <ul>
 *   <li><b>Steer feedback: RemoteCANcoder.</b> The generator defaults to FusedCANcoder, which is a
 *       Phoenix Pro feature. This robot is not Pro licensed and the steer config apply is rejected
 *       with UsingProFeatureOnUnlicensedDevice, taking the current limits down with it.</li>
 *   <li><b>Drive supply limit: 35 A.</b> The generator sets no drive limits (factory default is
 *       70 A supply). Four drive motors at 70 A dragged the battery to 6.9 V (mttd p8). 20 A was too
 *       far the other way: in mttd q8 the drive motors sat on the limit 77% of teleop and the robot
 *       averaged 1.4 m/s at full stick with the battery never below 9.1 V.
 *       Note the swerve API replaces the drive <i>stator</i> limit with the slip current from
 *       TunerConstants, so only the supply limit is set here.</li>
 *   <li><b>Steer limits: 40 A stator, 20 A supply.</b> The generator uses 60 A stator and no supply
 *       limit. Steering needs little torque, so lower limits cost nothing and reduce brownouts.</li>
 * </ul>
 */
public final class SwerveConfigOverrides {
    public static final SteerFeedbackType kSteerFeedback = SteerFeedbackType.RemoteCANcoder;
    public static final double kDriveSupplyLimitAmps = 35;
    public static final double kSteerStatorLimitAmps = 40;
    public static final double kSteerSupplyLimitAmps = 20;

    private static final double kAmpsTolerance = 0.5;
    private static final double kConfigReadTimeoutSec = 0.05;
    /** Check one motor every this many periodic calls (0.2 s at 50 Hz), so a full pass takes ~1.6 s. */
    private static final int kTicksBetweenChecks = 10;
    private static final String[] kModuleNames = { "FrontLeft", "FrontRight", "BackLeft", "BackRight" };

    /**
     * Applies the team's overrides to the generated module constants. Call this on the constants before
     * handing them to the drivetrain constructor. The generated initial configs are edited in place so
     * everything else the generator put in them is kept.
     */
    public static SwerveModuleConstants<?, ?, ?>[] apply(SwerveModuleConstants<?, ?, ?>... modules) {
        for (SwerveModuleConstants<?, ?, ?> module : modules) {
            if (module.DriveMotorInitialConfigs instanceof TalonFXConfiguration drive) {
                drive.CurrentLimits.SupplyCurrentLimit = kDriveSupplyLimitAmps;
                drive.CurrentLimits.SupplyCurrentLimitEnable = true;
            }
            if (module.SteerMotorInitialConfigs instanceof TalonFXConfiguration steer) {
                steer.CurrentLimits.StatorCurrentLimit = kSteerStatorLimitAmps;
                steer.CurrentLimits.StatorCurrentLimitEnable = true;
                steer.CurrentLimits.SupplyCurrentLimit = kSteerSupplyLimitAmps;
                steer.CurrentLimits.SupplyCurrentLimitEnable = true;
            }
            module.withFeedbackSource(kSteerFeedback);
        }
        return modules;
    }

    private final SwerveModule<TalonFX, TalonFX, CANcoder>[] modules;
    /** One entry per motor (drive, steer, drive, steer, ...); null means that motor checked out. */
    private final String[] mismatches;
    private final Alert configAlert = new Alert("", AlertType.kError);
    private int tick = 0;
    private int nextMotor = 0;
    private boolean fullPassDone = false;

    public SwerveConfigOverrides(SwerveModule<TalonFX, TalonFX, CANcoder>[] modules) {
        this.modules = modules;
        this.mismatches = new String[modules.length * 2];
    }

    /**
     * Reads one motor's live config back and compares it to the overrides. Call every loop while the
     * robot is disabled. Each call may block for up to {@link #kConfigReadTimeoutSec} on the CAN bus,
     * which is why it is spread out and not run while enabled.
     */
    public void verifyPeriodic() {
        if (++tick < kTicksBetweenChecks) {
            return;
        }
        tick = 0;

        int moduleIndex = nextMotor / 2;
        boolean isSteer = nextMotor % 2 == 1;
        SwerveModule<TalonFX, TalonFX, CANcoder> module = modules[moduleIndex];
        TalonFX motor = isSteer ? module.getSteerMotor() : module.getDriveMotor();
        String label = kModuleNames[moduleIndex] + (isSteer ? " steer" : " drive")
            + " motor (CAN " + motor.getDeviceID() + ")";
        mismatches[nextMotor] = check(motor, isSteer, module.getEncoder().getDeviceID(), label);

        nextMotor = (nextMotor + 1) % mismatches.length;
        if (nextMotor == 0) {
            fullPassDone = true;
        }
        publish();
    }

    private String check(TalonFX motor, boolean isSteer, int encoderId, String label) {
        // A config read from a motor that is off the bus waits the full timeout and stalls the robot
        // loop every time. isConnected() only looks at signals already received, so it never blocks.
        if (!motor.isConnected()) {
            return label + ": not responding on CAN, config not checked";
        }
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        StatusCode status = motor.getConfigurator().refresh(cfg, kConfigReadTimeoutSec);
        if (!status.isOK()) {
            return label + ": could not read config (" + status.getName() + ")";
        }

        List<String> problems = new ArrayList<>();
        CurrentLimitsConfigs limits = cfg.CurrentLimits;
        if (isSteer) {
            expectAmps(problems, "stator limit", limits.StatorCurrentLimit, kSteerStatorLimitAmps);
            expectAmps(problems, "supply limit", limits.SupplyCurrentLimit, kSteerSupplyLimitAmps);
            if (!limits.StatorCurrentLimitEnable) {
                problems.add("stator limit disabled");
            }
            if (!limits.SupplyCurrentLimitEnable) {
                problems.add("supply limit disabled");
            }
            if (cfg.Feedback.FeedbackSensorSource != FeedbackSensorSourceValue.RemoteCANcoder) {
                problems.add("feedback is " + cfg.Feedback.FeedbackSensorSource + " not RemoteCANcoder");
            }
            if (cfg.Feedback.FeedbackRemoteSensorID != encoderId) {
                problems.add("feedback CANcoder is ID " + cfg.Feedback.FeedbackRemoteSensorID
                    + " not " + encoderId);
            }
        } else {
            expectAmps(problems, "supply limit", limits.SupplyCurrentLimit, kDriveSupplyLimitAmps);
            if (!limits.SupplyCurrentLimitEnable) {
                problems.add("supply limit disabled");
            }
        }
        return problems.isEmpty() ? null : label + ": " + String.join(", ", problems);
    }

    private static void expectAmps(List<String> problems, String what, double actual, double expected) {
        if (Math.abs(actual - expected) > kAmpsTolerance) {
            problems.add(what + " is " + Math.round(actual) + " A not " + Math.round(expected) + " A");
        }
    }

    private void publish() {
        List<String> active = new ArrayList<>();
        for (String m : mismatches) {
            if (m != null) {
                active.add(m);
            }
        }
        boolean verified = fullPassDone && active.isEmpty();
        String text = String.join("; ", active);
        configAlert.setText("Swerve config NOT applied. Redeploy or power cycle. " + text);
        configAlert.set(!active.isEmpty());
        Logger.recordOutput("Drive/Config/Verified", verified);
        Logger.recordOutput("Drive/Config/Mismatches", text);
    }
}
