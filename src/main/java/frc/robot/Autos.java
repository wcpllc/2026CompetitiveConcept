package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;
import java.util.stream.IntStream;

public class Autos {

    public record AutoRoutine(boolean canMirror, String displayLabel, List<String> pathNames) {}

    private static class AutoPathOption {
        final AutoRoutine routine;
        final boolean mirrored;

        AutoPathOption(AutoRoutine routine, boolean mirrored) {
            this.routine = routine;
            this.mirrored = mirrored;
        }

        @Override
        public String toString() {
            return routine.displayLabel() + (mirrored ? " (Mirrored)" : "");
        }
    }

    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds =
        new SwerveRequest.ApplyRobotSpeeds();

    private final SendableChooser<AutoPathOption> chooser = new SendableChooser<>();

    private final FollowPath.Builder builder;           // resets pose
    private final FollowPath.Builder continuationBuilder; // no pose reset

    public Autos(CommandSwerveDrivetrain swerve) {
        chooser.setDefaultOption("None", null);
        populateChooser();
        SmartDashboard.putData("Auto Mode", chooser);

        // Starting guesses only; tune on the robot
        PIDController translationPid = new PIDController(4.0, 0.0, 0.0);
        PIDController rotationPid    = new PIDController(4.0, 0.0, 0.0);
        PIDController crossTrackPid  = new PIDController(2.0, 0.0, 0.0);

        builder = new FollowPath.Builder(
            swerve,
            () -> swerve.getState().Pose,
            () -> swerve.getState().Speeds,  // robot-relative
            speeds -> swerve.setControl(applyRobotSpeeds.withSpeeds(speeds)),
            translationPid,
            rotationPid,
            crossTrackPid
        )
        .withShouldMirror(() -> {
            AutoPathOption selected = chooser.getSelected();
            return selected != null && selected.mirrored;
        })
        .withDefaultShouldFlip()
        .withTRatioBasedTranslationHandoffs(true)
        .withPoseReset(swerve::resetPose);

        // Second builder needs its OWN PID objects (controllers can't be shared concurrently)
        continuationBuilder = new FollowPath.Builder(
            swerve,
            () -> swerve.getState().Pose,
            () -> swerve.getState().Speeds,
            speeds -> swerve.setControl(applyRobotSpeeds.withSpeeds(speeds)),
            new PIDController(4.0, 0.0, 0.0),
            new PIDController(4.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0)
        )
        .withShouldMirror(() -> {
            AutoPathOption selected = chooser.getSelected();
            return selected != null && selected.mirrored;
        })
        .withDefaultShouldFlip()
        .withTRatioBasedTranslationHandoffs(true);
    }

    private void populateChooser() {
        for (AutoRoutine routine : AUTO_ROUTINES) {
            AutoPathOption normal = new AutoPathOption(routine, false);
            chooser.addOption(normal.toString(), normal);
            if (routine.canMirror()) {
                AutoPathOption mirrored = new AutoPathOption(routine, true);
                chooser.addOption(mirrored.toString(), mirrored);
            }
        }
    }
    public Command getAutonomousCommand() {
        AutoPathOption selected = chooser.getSelected();
        if (selected == null || selected.routine.pathNames().isEmpty()) {
            return Commands.none();
        }
        List<Path> paths = selected.routine.pathNames().stream().map(Path::new).toList();
        return Commands.sequence(
            IntStream.range(0, paths.size())
                .mapToObj(i -> (i == 0 ? builder : continuationBuilder).build(paths.get(i)))
                .toArray(Command[]::new));
    }

    public static final List<AutoRoutine> AUTO_ROUTINES = 
        List.of(    
        new AutoRoutine(
            true, 
                "9584-Madtown-auto", List.of("9584-Madtown-auto"))
        // add your routines here
        
    );
}
