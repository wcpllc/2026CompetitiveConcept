package frc.robot.sim;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Time;
import static edu.wpi.first.units.Units.*;

public abstract class SimProfile {
    private boolean running = false;
    private double lastTime = 0;

    public abstract void run();

    protected Time getPeriod() {
        if (!running) {
            lastTime = Utils.getCurrentTimeSeconds();
            running = true;
        }

        double now = Utils.getCurrentTimeSeconds();
        double period = now - lastTime;
        lastTime = now;

        return Seconds.of(period);
    }
}
