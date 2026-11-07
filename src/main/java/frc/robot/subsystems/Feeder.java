package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
import frc.robot.sim.SimDeviceRegistrar;

public class Feeder extends SubsystemBase {
    public enum Speed {
        FEED(-7500),
        OUTTAKE(7500);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    private final TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double cachedRPM, cachedStatorCurrent, cachedSupplyCurrent, cachedTemp;

    public Feeder() {
        motor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(25))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(1)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
        SimDeviceRegistrar.registerTalonFX(motor);
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        motor.setControl(
            velocityRequest
                .withVelocity(speed.angularVelocity())
        );
    }

    public void setPercentOutput(double percentOutput) {
        motor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public Command feedCommand() {
        System.out.println("===========Feed Command");
        return startEnd(
            () -> {
                System.out.println("feeding fuel to shooter");
                set(Speed.FEED);
                }, 
            () -> {
                System.out.println("stopping feed");
                setPercentOutput(0);
                });
    }

    public Command outtakeCommand() {
        System.out.println("===========Outtake Command");
        return startEnd(
            () -> {
                System.out.println("outtaking");
                set(Speed.OUTTAKE);
                }, 
            () -> {
                System.out.println("stopping outtake");
                setPercentOutput(0);
                });
    }

    @Override
    public void periodic() {
        cachedRPM = motor.getVelocity().getValue().in(RPM);
        cachedStatorCurrent = motor.getStatorCurrent().getValueAsDouble();
        cachedSupplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
        cachedTemp = motor.getDeviceTemp().getValueAsDouble();

        Logger.recordOutput("Feeder/SupplyCurrent", cachedSupplyCurrent);
        Logger.recordOutput("Feeder/Temp", cachedTemp);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", () -> cachedRPM, null);
        builder.addDoubleProperty("Feeder Stator Current", () -> cachedStatorCurrent, null);
        builder.addDoubleProperty("Feeder Supply Current", () -> cachedSupplyCurrent, null);
    }
}
