package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Logs the health of every swerve module so a drivetrain problem can be traced to one part.
 *
 * <p>For each of the four modules this records, every robot loop:
 * <ul>
 *   <li>Drive motor: supply current, stator current, applied voltage, temperature, faults</li>
 *   <li>Steer motor: the same, plus how far the wheel is from its target angle</li>
 *   <li>CANcoder: absolute angle, magnet health, faults</li>
 *   <li>Whether each device is answering on the CAN bus at all</li>
 * </ul>
 *
 * <p>Everything lands under {@code Drive/Modules/<ModuleName>/...} in the log, and any active
 * fault also shows up as a dashboard alert. See {@code docs/drivetrain-log-guide.md} for how to
 * read these values.
 */
public class SwerveModuleDiagnostics {
    private static final String[] kModuleNames = { "FrontLeft", "FrontRight", "BackLeft", "BackRight" };
    private static final String kLogPrefix = "Drive/Modules/";

    /** How often the motors send current, voltage, and temperature over CAN. */
    private static final double kSignalUpdateFrequencyHz = 50.0;

    /** One named fault flag on a device. */
    private record Fault(String name, StatusSignal<Boolean> signal) {}

    /** All the signals and alerts for one module. */
    private static class ModuleDiagnostics {
        final String name;
        final SwerveModule<TalonFX, TalonFX, CANcoder> module;

        final StatusSignal<?> driveSupplyCurrent;
        final StatusSignal<?> driveStatorCurrent;
        final StatusSignal<?> driveVoltage;
        final StatusSignal<?> driveTemp;

        final StatusSignal<?> steerSupplyCurrent;
        final StatusSignal<?> steerStatorCurrent;
        final StatusSignal<?> steerVoltage;
        final StatusSignal<?> steerTemp;
        final StatusSignal<Double> steerClosedLoopError;

        final StatusSignal<?> encoderAbsolutePosition;
        final StatusSignal<MagnetHealthValue> encoderMagnetHealth;

        final List<Fault> driveFaults;
        final List<Fault> steerFaults;
        final List<Fault> encoderFaults;

        final String driveLabel;
        final String steerLabel;
        final String encoderLabel;
        final Alert driveAlert;
        final Alert steerAlert;
        final Alert encoderAlert;

        ModuleDiagnostics(String name, SwerveModule<TalonFX, TalonFX, CANcoder> module) {
            this.name = name;
            this.module = module;
            TalonFX drive = module.getDriveMotor();
            TalonFX steer = module.getSteerMotor();
            CANcoder encoder = module.getEncoder();

            driveSupplyCurrent = drive.getSupplyCurrent();
            driveStatorCurrent = drive.getStatorCurrent();
            driveVoltage = drive.getMotorVoltage();
            driveTemp = drive.getDeviceTemp();

            steerSupplyCurrent = steer.getSupplyCurrent();
            steerStatorCurrent = steer.getStatorCurrent();
            steerVoltage = steer.getMotorVoltage();
            steerTemp = steer.getDeviceTemp();
            steerClosedLoopError = steer.getClosedLoopError();

            encoderAbsolutePosition = encoder.getAbsolutePosition();
            encoderMagnetHealth = encoder.getMagnetHealth();

            driveFaults = motorFaults(drive);
            steerFaults = motorFaults(steer);
            encoderFaults = List.of(
                new Fault("BadMagnet", encoder.getFault_BadMagnet()),
                new Fault("Hardware", encoder.getFault_Hardware()),
                new Fault("Undervoltage", encoder.getFault_Undervoltage()),
                new Fault("BootDuringEnable", encoder.getFault_BootDuringEnable())
            );

            driveLabel = name + " drive motor (CAN " + drive.getDeviceID() + ")";
            steerLabel = name + " steer motor (CAN " + steer.getDeviceID() + ")";
            encoderLabel = name + " CANcoder (CAN " + encoder.getDeviceID() + ")";
            driveAlert = new Alert(driveLabel, AlertType.kError);
            steerAlert = new Alert(steerLabel, AlertType.kError);
            encoderAlert = new Alert(encoderLabel, AlertType.kError);
        }

        /** The faults on a TalonFX that matter for a swerve module. */
        private static List<Fault> motorFaults(TalonFX motor) {
            return List.of(
                new Fault("Hardware", motor.getFault_Hardware()),
                new Fault("Undervoltage", motor.getFault_Undervoltage()),
                new Fault("BootDuringEnable", motor.getFault_BootDuringEnable()),
                new Fault("RemoteSensorDataInvalid", motor.getFault_RemoteSensorDataInvalid()),
                new Fault("RemoteSensorReset", motor.getFault_RemoteSensorReset()),
                new Fault("StatorCurrentLimit", motor.getFault_StatorCurrLimit()),
                new Fault("SupplyCurrentLimit", motor.getFault_SupplyCurrLimit()),
                new Fault("DeviceTemp", motor.getFault_DeviceTemp()),
                new Fault("BridgeBrownout", motor.getFault_BridgeBrownout())
            );
        }

        /** Signals that should be refreshed at the fast rate. */
        List<BaseStatusSignal> measurementSignals() {
            return List.of(
                driveSupplyCurrent, driveStatorCurrent, driveVoltage, driveTemp,
                steerSupplyCurrent, steerStatorCurrent, steerVoltage, steerTemp, steerClosedLoopError,
                encoderAbsolutePosition, encoderMagnetHealth
            );
        }

        /** Every signal this module reads, measurements and faults together. */
        List<BaseStatusSignal> allSignals() {
            List<BaseStatusSignal> signals = new ArrayList<>(measurementSignals());
            for (Fault f : driveFaults) signals.add(f.signal());
            for (Fault f : steerFaults) signals.add(f.signal());
            for (Fault f : encoderFaults) signals.add(f.signal());
            return signals;
        }

        void log() {
            String prefix = kLogPrefix + name + "/";

            // What the module was told to do versus what it actually did.
            SwerveModuleState target = module.getTargetState();
            SwerveModuleState measured = module.getCurrentState();
            Logger.recordOutput(prefix + "TargetAngleDegrees", target.angle.getDegrees());
            Logger.recordOutput(prefix + "MeasuredAngleDegrees", measured.angle.getDegrees());
            Logger.recordOutput(prefix + "TargetSpeedMetersPerSec", target.speedMetersPerSecond);
            Logger.recordOutput(prefix + "MeasuredSpeedMetersPerSec", measured.speedMetersPerSecond);

            // Drive motor
            Logger.recordOutput(prefix + "Drive/SupplyCurrentAmps", driveSupplyCurrent.getValueAsDouble());
            Logger.recordOutput(prefix + "Drive/StatorCurrentAmps", driveStatorCurrent.getValueAsDouble());
            Logger.recordOutput(prefix + "Drive/VoltageVolts", driveVoltage.getValueAsDouble());
            Logger.recordOutput(prefix + "Drive/TempCelsius", driveTemp.getValueAsDouble());
            Logger.recordOutput(prefix + "Drive/Connected", driveSupplyCurrent.getStatus().isOK());
            boolean driveFaulted = logFaults(prefix + "Drive/", driveFaults, driveLabel, driveAlert, driveSupplyCurrent);

            // Steer motor
            Logger.recordOutput(prefix + "Steer/SupplyCurrentAmps", steerSupplyCurrent.getValueAsDouble());
            Logger.recordOutput(prefix + "Steer/StatorCurrentAmps", steerStatorCurrent.getValueAsDouble());
            Logger.recordOutput(prefix + "Steer/VoltageVolts", steerVoltage.getValueAsDouble());
            Logger.recordOutput(prefix + "Steer/TempCelsius", steerTemp.getValueAsDouble());
            // Closed-loop error is in rotations of the CANcoder; convert to degrees so it is easy to read.
            Logger.recordOutput(prefix + "Steer/ErrorDegrees", steerClosedLoopError.getValueAsDouble() * 360.0);
            Logger.recordOutput(prefix + "Steer/Connected", steerSupplyCurrent.getStatus().isOK());
            boolean steerFaulted = logFaults(prefix + "Steer/", steerFaults, steerLabel, steerAlert, steerSupplyCurrent);

            // CANcoder
            MagnetHealthValue magnetHealth = encoderMagnetHealth.getValue();
            Logger.recordOutput(prefix + "Encoder/AbsoluteDegrees", encoderAbsolutePosition.getValueAsDouble() * 360.0);
            Logger.recordOutput(prefix + "Encoder/MagnetHealth", magnetHealth.toString());
            Logger.recordOutput(prefix + "Encoder/MagnetOk", magnetHealth == MagnetHealthValue.Magnet_Green);
            Logger.recordOutput(prefix + "Encoder/Connected", encoderAbsolutePosition.getStatus().isOK());
            boolean encoderFaulted = logFaults(prefix + "Encoder/", encoderFaults, encoderLabel, encoderAlert, encoderAbsolutePosition);

            Logger.recordOutput(prefix + "HasFault", driveFaulted || steerFaulted || encoderFaulted);
        }

        /**
         * Logs which faults are active on one device and raises its alert if any are.
         *
         * @return true if the device has an active fault or is not responding
         */
        private static boolean logFaults(String prefix, List<Fault> faults, String label, Alert alert, StatusSignal<?> livenessSignal) {
            StringBuilder active = new StringBuilder();
            for (Fault fault : faults) {
                if (fault.signal().getValue()) {
                    if (active.length() > 0) active.append(", ");
                    active.append(fault.name());
                }
            }
            boolean connected = livenessSignal.getStatus().isOK();
            if (!connected) {
                if (active.length() > 0) active.append(", ");
                active.append("NotResponding");
            }
            String faultText = active.toString();
            boolean faulted = !faultText.isEmpty();

            Logger.recordOutput(prefix + "Faults", faultText);
            Logger.recordOutput(prefix + "HasFault", faulted);
            alert.setText(faulted ? label + ": " + faultText : label);
            alert.set(faulted);
            return faulted;
        }
    }

    private final ModuleDiagnostics[] modules;
    private final List<BaseStatusSignal> allSignals = new ArrayList<>();

    public SwerveModuleDiagnostics(SwerveModule<TalonFX, TalonFX, CANcoder>[] swerveModules) {
        modules = new ModuleDiagnostics[swerveModules.length];
        List<BaseStatusSignal> measurementSignals = new ArrayList<>();
        for (int i = 0; i < swerveModules.length; i++) {
            String name = i < kModuleNames.length ? kModuleNames[i] : "Module" + i;
            modules[i] = new ModuleDiagnostics(name, swerveModules[i]);
            measurementSignals.addAll(modules[i].measurementSignals());
            allSignals.addAll(modules[i].allSignals());
        }
        // Fault flags keep their default (slow) update rate; measurements are sped up so the log
        // shows what happened within one or two robot loops.
        BaseStatusSignal.setUpdateFrequencyForAll(kSignalUpdateFrequencyHz, measurementSignals);
    }

    /** Call once per robot loop. Reads the latest values from the CAN bus and logs them. */
    public void logPeriodic() {
        // refreshAll reads the most recent value received for every signal in one call. It does not
        // wait on the bus, so this is cheap even with ~130 signals.
        BaseStatusSignal.refreshAll(allSignals);
        boolean anyFault = false;
        for (ModuleDiagnostics module : modules) {
            module.log();
        }
        for (ModuleDiagnostics module : modules) {
            anyFault |= module.driveAlert.get() || module.steerAlert.get() || module.encoderAlert.get();
        }
        Logger.recordOutput("Drive/AnyModuleFault", anyFault);
    }
}
