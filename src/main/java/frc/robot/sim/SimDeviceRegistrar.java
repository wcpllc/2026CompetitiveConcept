package frc.robot.sim;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;

/** Utility helpers for registering simulated CTRE devices. */
public final class SimDeviceRegistrar {
    private static final MomentOfInertia kDefaultRotorInertia = KilogramSquareMeters.of(0.00032);

    private SimDeviceRegistrar() {}

    /** Registers a Talon FX with a default Kraken X60 rotor inertia model. */
    public static void registerTalonFX(TalonFX talonFX) {
        registerTalonFX(talonFX, kDefaultRotorInertia);
    }

    /** Registers a Talon FX with the provided inertia for simulation updates. */
    public static void registerTalonFX(TalonFX talonFX, MomentOfInertia rotorInertia) {
        if (!RobotBase.isSimulation()) {
            return;
        }

        PhysicsSim.getInstance().addTalonFX(talonFX, rotorInertia);
    }
}
