package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 13;
    public static final int kIntakeRollers = 14;
    public static final int kFloor = 12;
    public static final int kFeeder = 15;
    public static final int kShooterLeft = 11;
    public static final int kShooterMiddle = 10;
    public static final int kShooterRight = 9;
    public static final int kHanger = 16;
    public static final int kHoodMotor = 24;

    // PWM port
    public static final int leftServoPort = 7;
    public static final int rightServoPort = 8;
}
