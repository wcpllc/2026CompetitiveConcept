package frc.robot.utils.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/**
 * Basic robot dimensions that are reused by the simulator when mapping collisions and intakes.
 */
public final class Dimensions {
    public static final double FULL_LENGTH = SimSwerveConstants.BUMPER_LENGTH_X.in(Meters);
    public static final double FULL_WIDTH = SimSwerveConstants.BUMPER_LENGTH_Y.in(Meters);
    public static final double BUMPER_HEIGHT = Inches.of(7).in(Meters);

    private Dimensions() {}
}
