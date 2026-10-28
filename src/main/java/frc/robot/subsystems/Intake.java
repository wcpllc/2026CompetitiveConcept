package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
import frc.robot.sim.SimDeviceRegistrar;

public class Intake extends SubsystemBase {
    public enum Speed {
        DEFAULT(0),
        INTAKE(0.9),
        OUTTAKE(-0.9);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum Position {
        HOMED(110),
        STOWED(32),
        INTAKE(108), // red = 108, blue = 105.5
        AGITATE(80);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPivotReduction = 62.5;
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(5);

    private final TalonFX pivotMotor, rollerMotor;
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
    private Speed currentSpeed = Speed.DEFAULT;

    private static final double kHomingPercentOutput = 0.1;
    private static final double kHomingCurrentThresholdAmps = 6.0;
    private static final double kHomingTimeoutSeconds = 1.5;

    private boolean isHomed = false;

    private double cachedPivotAngleDeg;
    private double cachedRollerRPM;
    private double cachedPivotSupplyCurrent;
    private double cachedRollerSupplyCurrent;
    private double cachedPivotTemp;
    private double cachedRollerTemp;

    public Intake() {
        pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kRoboRioCANBus);
        rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);
        configurePivotMotor();
        configureRollerMotor();
        SimDeviceRegistrar.registerTalonFX(pivotMotor);
        SimDeviceRegistrar.registerTalonFX(rollerMotor);
        SmartDashboard.putData(this);
    }

    private void configurePivotMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(35))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(20))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        pivotMotor.getConfigurator().apply(config);
    }

    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(35))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(20))
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);
    }

    private boolean isPositionWithinTolerance() {
        final Angle currentPosition = pivotMotor.getPosition().getValue();
        final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(targetPosition, kPositionTolerance);
    }

    private void setPivotPercentOutput(double percentOutput) {
        pivotMotor.setControl(
            pivotVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public void set(Position position) {
        System.out.println("Setting intake position to " + position);
        pivotMotor.setControl(
            pivotMotionMagicRequest
                .withPosition(position.angle())
        );
        System.out.println("Intake position set");
    }

    public void set(Speed speed) {
        System.out.println("Setting intake speed to "  + speed);
        currentSpeed = speed;
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(speed.voltage())
        );
        System.out.println("Intake speed set");
    }

    public boolean isIntaking() {
        return currentSpeed == Speed.INTAKE;
    }

    public Command intakeCommand() {
        System.out.println("========Intake Command");
        return startEnd(
            () -> {
                System.out.println("Starting Intake");
                set(Position.INTAKE);
                set(Speed.INTAKE);
            },
            () -> {
                System.out.println("Stopping Intake");
                set(Speed.DEFAULT);
            }
        );
    }

    public Command outtakeCommand() {
        System.out.println("========Outtake Command");
        return startEnd(
            () -> {
                System.out.println("Starting Outtake");
                set(Speed.OUTTAKE);
            },
            () -> {
                System.out.println("Stopping Outtake");
                set(Speed.DEFAULT);
            }
        );
    }

    public Command agitateCommand() {
        return runOnce(() -> set(Speed.DEFAULT))
            .andThen(runOnce(() -> setPivotPercentOutput(0.2)))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    //Commands.waitUntil(this::isPositionWithinTolerance),
                    Commands.waitSeconds(0.3),
                    runOnce(() -> set(Position.INTAKE)),
                    //Commands.waitUntil(this::isPositionWithinTolerance),
                    Commands.waitSeconds(0.3)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                set(Position.INTAKE);
                set(Speed.DEFAULT);
            });
    }

    public Command homingCommand() {
        final BooleanSupplier hasHitHardstop =
            () -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > kHomingCurrentThresholdAmps;
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(kHomingPercentOutput)),
            Commands.waitUntil(hasHitHardstop).withTimeout(kHomingTimeoutSeconds),
            runOnce(() -> {
                if (hasHitHardstop.getAsBoolean()) {
                    pivotMotor.setPosition(Position.HOMED.angle());
                    isHomed = true;
                    set(Position.STOWED);
                } else {
                    DriverStation.reportWarning("Intake homing timed out before hitting the hard stop", false);
                }
            })
        )
        .finallyDo(interrupted -> setPivotPercentOutput(0))
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    @Override
    public void periodic() {
        cachedPivotAngleDeg = pivotMotor.getPosition().getValue().in(Degrees);
        cachedRollerRPM = rollerMotor.getVelocity().getValue().in(RPM);
        cachedPivotSupplyCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
        cachedRollerSupplyCurrent = rollerMotor.getSupplyCurrent().getValueAsDouble();
        cachedPivotTemp = pivotMotor.getDeviceTemp().getValueAsDouble();
        cachedRollerTemp = rollerMotor.getDeviceTemp().getValueAsDouble();

        Logger.recordOutput("Intake/PivotSupplyCurrent", cachedPivotSupplyCurrent);
        Logger.recordOutput("Intake/RollerSupplyCurrent", cachedRollerSupplyCurrent);
        Logger.recordOutput("Intake/PivotTemp", cachedPivotTemp);
        Logger.recordOutput("Intake/RollerTemp", cachedRollerTemp);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Angle (degrees)", () -> cachedPivotAngleDeg, null);
        builder.addDoubleProperty("RPM", () -> cachedRollerRPM, null);
        builder.addDoubleProperty("Pivot Supply Current", () -> cachedPivotSupplyCurrent, null);
        builder.addDoubleProperty("Roller Supply Current", () -> cachedRollerSupplyCurrent, null);
    }
}
