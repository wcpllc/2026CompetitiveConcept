package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.units.measure.MomentOfInertia;
import static edu.wpi.first.units.Units.*;

public class TalonFXSimProfile extends SimProfile {
    private DCMotorSim motorSim;
    private TalonFXSimState talonFXSim;
    private static final double kMotorResistance = 0.002;
    
    public TalonFXSimProfile(TalonFX talonFX, MomentOfInertia rotorInertia) {
        motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), rotorInertia.in(KilogramSquareMeters), 1),
            DCMotor.getKrakenX60(1)
        );
        talonFXSim = talonFX.getSimState();
    }

    @Override
    public void run() {
        motorSim.setInputVoltage(talonFXSim.getMotorVoltage());
        motorSim.update(getPeriod().in(Seconds));
        double rotations = motorSim.getAngularPositionRad() / (2 * Math.PI);
        double rotationsPerSecond = motorSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        talonFXSim.setRawRotorPosition(rotations);
        talonFXSim.setRotorVelocity(rotationsPerSecond);
        talonFXSim.setSupplyVoltage(12 - talonFXSim.getSupplyCurrent() * kMotorResistance);
    }
}
