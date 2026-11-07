package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Collections;
import java.util.List;
import java.util.stream.IntStream;
import lombok.Getter;

public class Autos {

  /**
   * @param displayLabel what driverstation displays as the auto's name
   * @param pathNames what paths (in order) make up the auto
   */
  public record AutoRoutine(boolean canMirror, String displayLabel, List<String> pathNames) {
    public AutoRoutine {
      pathNames = List.copyOf(pathNames);
    }
  }

  private static class AutoPathOption {
    public final AutoRoutine routine;
    public final boolean mirrored;

    public AutoPathOption(AutoRoutine routine, boolean mirrored) {
      this.routine = routine;
      this.mirrored = mirrored;
    }

    @Override
    public String toString() {
      return routine.displayLabel() + (mirrored ? " (Mirrored)" : "");
    }
  }

  private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  @Getter private final SendableChooser<AutoPathOption> chooser = new SendableChooser<>();

  private final FollowPath.Builder pathBuilderWithStartPoseReset;
  private final FollowPath.Builder pathBuilderContinuation;
  private AutoPathOption cachedSelectedAuto = null;
  private List<Path> cachedPathSegments = Collections.<Path>emptyList();
  private Pose2d cachedAutonomousStartPose = null;
  private boolean selectedAutoIsCached = false;

  public Autos(CommandSwerveDrivetrain drivetrain) {

    chooser.setDefaultOption("None", null);

    // Build chooser FIRST so builder can reference it
    populateChooser();

    pathBuilderWithStartPoseReset = newPathBuilder(drivetrain).withPoseReset(drivetrain::resetPose);
    pathBuilderContinuation = newPathBuilder(drivetrain);

    SmartDashboard.putData("Auto Mode", chooser);
  }

  private FollowPath.Builder newPathBuilder(CommandSwerveDrivetrain drivetrain) {
    return new FollowPath.Builder(
            drivetrain,
            () -> drivetrain.getState().Pose,
            () -> drivetrain.getState().Speeds,
            speeds -> drivetrain.setControl(pathApplyRobotSpeeds.withSpeeds(speeds)),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0))
        .withShouldMirror(
            () -> {
              AutoPathOption selected = chooser.getSelected();
              return selected != null && selected.mirrored;
            })
        .withDefaultShouldFlip();
  }

  private void populateChooser() {
    for (AutoRoutine routine : AUTO_ROUTINES) {
      AutoPathOption normal = new AutoPathOption(routine, false);
      chooser.addOption(normal.toString(), normal);
      if (routine.canMirror) {
        AutoPathOption mirrored = new AutoPathOption(routine, true);
        chooser.addOption(mirrored.toString(), mirrored);
      }
    }
  }

  public Command getAutonomousCommand() {
    updateSelectedAutoCache();
    return buildCommandFromCachedSegments();
  }

  public Pose2d getAutonomousStartPose() {
    updateSelectedAutoCache();
    return cachedAutonomousStartPose;
  }

  public boolean preloadSelectedAuto() {
    updateSelectedAutoCache();
    return selectedAutoIsCached;
  }

  public boolean isSelectedAutoCached() {
    return selectedAutoIsCached;
  }

  public String getSelectedAutoName() {
    AutoPathOption selected = chooser.getSelected();
    return selected == null ? "None" : selected.toString();
  }

  private void updateSelectedAutoCache() {
    AutoPathOption selected = chooser.getSelected();

    // if didn't change auto --> don't need to re-cache
    if (selected == cachedSelectedAuto && selectedAutoIsCached) {
      return;
    }

    // set flag --> so can see when not-ready/not-cached
    selectedAutoIsCached = false;
    cachedSelectedAuto = selected;

    if (selected == null) {
      cachedPathSegments = Collections.<Path>emptyList();
      cachedAutonomousStartPose = null;
      return;
    }

    cachedPathSegments = selected.routine.pathNames().stream().map(Path::new).toList();

    // empty auto
    if (cachedPathSegments.isEmpty()) {
      cachedAutonomousStartPose = null;
      return;
    }

    // extract start pose
    Path startPath = cachedPathSegments.get(0).copy();
    if (selected.mirrored) startPath.mirror();
    cachedAutonomousStartPose = startPath.getStartPose();

    // flip back boolean flag: caching process finished
    selectedAutoIsCached = true;
  }

  private Command buildCommandFromCachedSegments() {
    if (cachedPathSegments.isEmpty()) {
      return Commands.none();
    }

    return Commands.sequence(
        IntStream.range(0, cachedPathSegments.size())
            .mapToObj(
                i -> {
                  FollowPath.Builder builder =
                      i == 0 ? pathBuilderWithStartPoseReset : pathBuilderContinuation;
                  return builder.build(cachedPathSegments.get(i));
                })
            .toArray(Command[]::new));
  }

  public static final List<AutoRoutine> AUTO_ROUTINES =
      List.of(
          new AutoRoutine(
              false,
              "Do Nothing",
              List.of("do nothing")));
              
}
