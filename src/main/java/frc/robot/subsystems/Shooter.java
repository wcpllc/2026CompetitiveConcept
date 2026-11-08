package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
import frc.robot.sim.SimDeviceRegistrar;

public class Shooter extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double dashboardTargetRPM = 3750.0;

    private double cachedLeftRPM, cachedMiddleRPM, cachedRightRPM;
    private double cachedLeftStatorCurrent, cachedMiddleStatorCurrent, cachedRightStatorCurrent;
    private double cachedLeftSupplyCurrent, cachedMiddleSupplyCurrent, cachedRightSupplyCurrent;
    private double cachedLeftTemp, cachedMiddleTemp, cachedRightTemp;

    public Shooter() {
        leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANBus);
        middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANBus);
        rightMotor = new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANBus);
        motors = List.of(leftMotor, middleMotor, rightMotor);

        configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive, 60, 45);
        configureMotor(middleMotor, InvertedValue.CounterClockwise_Positive, 60, 45);
        configureMotor(rightMotor, InvertedValue.Clockwise_Positive, 60, 45);

        motors.forEach(SimDeviceRegistrar::registerTalonFX);
        SmartDashboard.putData(this);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection, double statorLimit, double supplyLimit) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(statorLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(supplyLimit))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(1.2)
                    .withKA(0.01)
                    .withKI(2)
                    .withKS(0.2)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }

    public void setRPM(double rpm) {
        for (final TalonFX motor : motors) {
            motor.setControl(
            velocityRequest
                .withVelocity(RPM.of(rpm))
            );
        }
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                voltageRequest
                    .withOutput(Volts.of(percentOutput * 12.0))
            );
        }
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public void setDashboardRPM(double rpm) {
        dashboardTargetRPM = rpm;
    }

    public double getDashboardRPM() {
        return dashboardTargetRPM;
    }

    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM)); 
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }

    @Override
    public void periodic() {
        cachedLeftRPM = leftMotor.getVelocity().getValue().in(RPM);
        cachedMiddleRPM = middleMotor.getVelocity().getValue().in(RPM);
        cachedRightRPM = rightMotor.getVelocity().getValue().in(RPM);
        cachedLeftStatorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
        cachedMiddleStatorCurrent = middleMotor.getStatorCurrent().getValueAsDouble();
        cachedRightStatorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
        cachedLeftSupplyCurrent = leftMotor.getSupplyCurrent().getValueAsDouble();
        cachedMiddleSupplyCurrent = middleMotor.getSupplyCurrent().getValueAsDouble();
        cachedRightSupplyCurrent = rightMotor.getSupplyCurrent().getValueAsDouble();
        cachedLeftTemp = leftMotor.getDeviceTemp().getValueAsDouble();
        cachedMiddleTemp = middleMotor.getDeviceTemp().getValueAsDouble();
        cachedRightTemp = rightMotor.getDeviceTemp().getValueAsDouble();

        Logger.recordOutput("Shooter/LeftSupplyCurrent", cachedLeftSupplyCurrent);
        Logger.recordOutput("Shooter/MiddleSupplyCurrent", cachedMiddleSupplyCurrent);
        Logger.recordOutput("Shooter/RightSupplyCurrent", cachedRightSupplyCurrent);
        Logger.recordOutput("Shooter/LeftTemp", cachedLeftTemp);
        Logger.recordOutput("Shooter/MiddleTemp", cachedMiddleTemp);
        Logger.recordOutput("Shooter/RightTemp", cachedRightTemp);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Left RPM", () -> cachedLeftRPM, null);
        builder.addDoubleProperty("Left Stator Current", () -> cachedLeftStatorCurrent, null);
        builder.addDoubleProperty("Left Supply Current", () -> cachedLeftSupplyCurrent, null);
        builder.addDoubleProperty("Middle RPM", () -> cachedMiddleRPM, null);
        builder.addDoubleProperty("Middle Stator Current", () -> cachedMiddleStatorCurrent, null);
        builder.addDoubleProperty("Middle Supply Current", () -> cachedMiddleSupplyCurrent, null);
        builder.addDoubleProperty("Right RPM", () -> cachedRightRPM, null);
        builder.addDoubleProperty("Right Stator Current", () -> cachedRightStatorCurrent, null);
        builder.addDoubleProperty("Right Supply Current", () -> cachedRightSupplyCurrent, null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
    }
}
