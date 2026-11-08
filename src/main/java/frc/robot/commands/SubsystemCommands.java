package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// BLine-Lib path following imports
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.Waypoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public final class SubsystemCommands {
    private final CommandSwerveDrivetrain swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        CommandSwerveDrivetrain swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        CommandSwerveDrivetrain swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger
    ) {
        this(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        System.out.println("=========Aim and Shoot Command=========");
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput); 
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
        return Commands.parallel(
             Commands.print("Aiming and shooting"),
             aimAndDriveCommand,
             Commands.waitSeconds(0.25)
                 .andThen(prepareShotCommand),
             Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
               .andThen(feed())
        );
    }

    public Command autoAim() {
        System.out.println("=========Running aim and drive command=========");
        return new AimAndDriveCommand(swerve, forwardInput, leftInput);
    }

    public Command lockWheels() {
        System.out.println("======== Lock Wheels =======");
        return swerve.lockWheelsCommand();
    }

    public Command shootManually() {
        System.out.println("========Shooting Manually=========");
        return
            Commands.print("Shooting manually") 
            .andThen(shooter.dashboardSpinUpCommand())
            .andThen(Commands.print("Done spinning shooter"))
            .andThen(feed())
            .andThen(Commands.print("Done feeding"))
            .handleInterrupt(() -> shooter.stop());
    }

    public Command shootManualWithShotTable() {
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
        return Commands.parallel(
            Commands.print("Preparing shot"),
            prepareShotCommand,
            Commands.waitUntil(() -> prepareShotCommand.isReadyToShoot())
                .andThen(feed())
        );
    }

    public Command shootManualForShootAuto() {
        System.out.println("========Shooting Manually=========");
        Command spinUp = Commands.startEnd(
            () -> shooter.setRPM(3000.0),
            shooter::stop,
            shooter
        );
        Command feedWhenReady = Commands.waitUntil(shooter::isVelocityWithinTolerance)
            .andThen(feedForShootAuto())
            .andThen(Commands.print("Done feeding"));
        return Commands.sequence(
            Commands.print("Shooting manually"),
            Commands.parallel(spinUp, feedWhenReady)
        );
    }

    public Command autoAlignClimbCommand() {
        System.out.println("=========Auto Align Climbing=========");
        return Commands.defer(
            () -> {
                Pose2d current = swerve.getPose();
                Pose2d goal = Landmarks.climbPose();
                Path path = new Path(
                    Constants.ClimbAlignment.kPathConstraints,
                    new Waypoint(current),
                    new Waypoint(goal));
                SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
                return new FollowPath.Builder(
                    swerve,
                    swerve::getPose,
                    () -> swerve.getState().Speeds,
                    speeds -> swerve.setControl(request.withSpeeds(speeds)),
                    new PIDController(5.0, 0.0, 0.0),
                    new PIDController(3.0, 0.0, 0.0),
                    new PIDController(2.0, 0.0, 0.0))
                    .withDefaultShouldFlip()
                    .build(path);
            },
            Set.of(swerve)
        );
    }

    private Command feed() {
        System.out.println("=========Feed========");
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand().alongWith(intake.agitateCommand()))
            )
        );
    }

    private Command feedForShootAuto() {
        System.out.println("=========Feed========");
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand())
            )
        );
    }
}
