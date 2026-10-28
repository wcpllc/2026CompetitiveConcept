package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class PrepareShotCommand extends Command {
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    private static final double kFixedHoodPosition = 0.9;

   // static {
    //    distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19)); //from wcp code
    //    distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40)); //might want to increase speed by like 20%ish
    //    distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));
   // }

    // TUNE THIS DURING COMP
     static {
         distanceToShotMap.put(Inches.of(16.5), new Shot(2950, 0.2));
         distanceToShotMap.put(Inches.of(43.5), new Shot(3000, 0.35)); 
         distanceToShotMap.put(Inches.of(117.5), new Shot(3500, 0.45));
         distanceToShotMap.put(Inches.of(139.9), new Shot(3300, 0.53));
         distanceToShotMap.put(Inches.of(156.0), new Shot(3650, 0.55));
         distanceToShotMap.put(Inches.of(175.0), new Shot(3750,0.60));
    }

    private final Shooter shooter;
    private final Hood hood;
    private final Supplier<Pose2d> robotPoseSupplier;

    public PrepareShotCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooter, hood);
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance();
    }

    private static final Translation2d kDefaultBlueHubPosition = new Translation2d(Inches.of(182.105), Inches.of(158.845));
    private static final Translation2d kDefaultRedHubPosition = new Translation2d(Inches.of(469.115), Inches.of(158.845));

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = DriverStation.getAlliance().get().equals(Alliance.Blue) ? kDefaultBlueHubPosition : kDefaultRedHubPosition;
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    @Override
    public void execute() {
        final Distance distanceToHub = getDistanceToHub();
        final Shot shot = distanceToShotMap.get(distanceToHub);
        shooter.setRPM(shot.shooterRPM);
        hood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}
