package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Hood extends SubsystemBase {
    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(40).per(Second);
    private static final double kMinPosition = 0.01;
    private static final double kMaxPosition = 0.8;
    private static final double kPositionTolerance = 0.01;

    private final Servo leftServo, rightServo;

    private double currentPosition = 0.01;
    private double targetPosition = 0.01;
    private Time lastUpdateTime = Seconds.of(0);

    public Hood() {
        leftServo = new Servo(Ports.leftServoPort);
        rightServo = new Servo(Ports.rightServoPort);
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        setPosition(currentPosition);
        SmartDashboard.putData(this);
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        leftServo.set(clampedPosition);
        rightServo.set(clampedPosition);
        targetPosition = clampedPosition;
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Position", () -> currentPosition, null);
        builder.addDoubleProperty("Left Servo Position", () -> leftServo.getPosition(), null);
        builder.addDoubleProperty("Right Servo Position", () -> rightServo.getPosition(), null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
    }
}