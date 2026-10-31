// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.PhysicsSim;
import frc.robot.utils.simulation.FuelSim;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathfindingCommand;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
    /** A USB stick plugged into the roboRIO shows up here. Preferred, because it is easy to pull. */
    private static final String kUsbLogPath = "/media/sda1/";
    /** Used when no USB stick is present. Download with AdvantageScope's File > Download Logs. */
    private static final String kOnboardLogPath = "/home/lvuser/logs/";
    private final Alert noUsbLogAlert = new Alert(
        "No USB stick found. Logs are going to " + kOnboardLogPath + " on the roboRIO instead.",
        AlertType.kWarning);

    private final RobotContainer m_robotContainer;
    private Command m_autonomousCommand;
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private Timer shiftTimer = new Timer(); //for shift tracking
    private boolean ourAllianceActive = false;
    
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        configureLogging();
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotController.setBrownoutVoltage(Volts.of(6.1));
    }
    
    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        if (RobotBase.isSimulation()) {
            PhysicsSim.getInstance().run();
        }
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        logPowerDistribution();
    }

    private void logPowerDistribution() {
        Logger.recordOutput("Power/TotalCurrent", pdh.getTotalCurrent());
        Logger.recordOutput("Power/Voltage", pdh.getVoltage());

        var canStatus = RobotController.getCANStatus();
        Logger.recordOutput("CAN/BusUtilization", canStatus.percentBusUtilization);
        Logger.recordOutput("CAN/BusOffCount", canStatus.busOffCount);
        Logger.recordOutput("CAN/TxFullCount", canStatus.txFullCount);
        Logger.recordOutput("CAN/ReceiveErrorCount", canStatus.receiveErrorCount);
        Logger.recordOutput("CAN/TransmitErrorCount", canStatus.transmitErrorCount);
    }

    @Override
    public void disabledInit() {
        m_autonomousCommand = null;
        //m_robotContainer.setSwerveDriveNeutralMode(NeutralModeValue.Coast);
        //m_robotContainer.setSwerveSteerNeutralMode(NeutralModeValue.Coast);
        m_robotContainer.requestSwerveIdle();
        m_robotContainer.logSwerveStickyFaults();
    }

    @Override
    public void disabledPeriodic() {
        // Reassert neutral modes while disabled in case firmware resets after brownouts
        //m_robotContainer.setSwerveDriveNeutralMode(NeutralModeValue.Coast);
        //m_robotContainer.setSwerveSteerNeutralMode(NeutralModeValue.Coast);
        m_robotContainer.requestSwerveIdle();
    }

    @Override
    public void autonomousInit() {
        //m_robotContainer.autonomousInit();
        //m_robotContainer.setSwerveDriveNeutralMode(NeutralModeValue.Brake);
        //m_robotContainer.setSwerveSteerNeutralMode(NeutralModeValue.Brake);
        // m_robotContainer.ensureSwervePoseSeeded();
        // if (RobotBase.isSimulation()) {
        //     m_robotContainer.resetFuelSim();
        // }
        m_robotContainer.resetPositionForAuto();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);;
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        //m_robotContainer.setSwerveDriveNeutralMode(NeutralModeValue.Brake);
        //m_robotContainer.setSwerveSteerNeutralMode(NeutralModeValue.Brake);
        //m_robotContainer.ensureSwervePoseSeeded();

        shiftTimer.restart(); //set timer to 0

        String gameData = DriverStation.getGameSpecificMessage(); //using FMS
        if (gameData.length() > 0) {
            char winner = gameData.charAt(0);
            Alliance ourAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            // r  means blue starts active (red won auto), b means red starts active (blue won auto)
            ourAllianceActive = (winner == 'R') == (ourAlliance == Alliance.Blue);
            //if red won and we are blue, we are active - true = true
            //if blue won and we are red, we are active - false = false
            //if blue won and we are blue we are inactive - if red won and we are red we are inactive - false = true, true = false
        } else {
            // Game data not received - default to false, driver should set manually
            ourAllianceActive = false;
            DriverStation.reportWarning("Game data not received! Shift tracking may be incorrect.", false);
        }
    }

    @Override
    public void teleopPeriodic() {
        double t = shiftTimer.get();
        // Shift windows (seconds into teleop)
        double[][] shifts = {{10, 35}, {35, 60}, {60, 85}, {85, 110}};
        boolean hubActive = false; //are we in our shift?
        double timeRemaining = 0;

        if(t < 10){ //transition
            hubActive = true;
            timeRemaining = 10-t;
        } else if (t >= 110){ //endgame
            hubActive = true;
            timeRemaining = 140-t;
        } else{ //use alliance shifts to get which one is our shift
            for (double[] shift : shifts) {
                if (t >= shift[0] && t < shift[1]) {
                    hubActive = ourAllianceActive;
                    timeRemaining = shift[1] - t;
                    break;
                }
            }
        }

        SmartDashboard.putBoolean("Hub Active", hubActive);
        SmartDashboard.putNumber("Time Left in Shift", timeRemaining);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        //m_robotContainer.setSwerveDriveNeutralMode(NeutralModeValue.Brake);
        //m_robotContainer.setSwerveSteerNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void simulationPeriodic() {
        FuelSim.getInstance().updateSim();
    }

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        if (RobotBase.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    private void configureLogging() {
        Logger.recordMetadata("ProjectName", "9584-wcp-code-2026");
        Logger.recordMetadata("Robot", RobotBase.isReal() ? "Real" : "Simulation");

        if (RobotBase.isReal()) {
            String logPath = chooseRealRobotLogPath();
            Logger.recordMetadata("LogPath", logPath);
            Logger.addDataReceiver(new WPILOGWriter(logPath));
        } else {
            try {
                Files.createDirectories(Path.of("logs"));
            } catch (IOException ex) {
                DriverStation.reportError("Failed to create logs directory: " + ex.getMessage(), false);
            }
            Logger.addDataReceiver(new WPILOGWriter("logs/"));
        }

        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
    }

    /** Picks the USB stick if one is mounted and writable, otherwise the roboRIO's own storage. */
    private String chooseRealRobotLogPath() {
        Path usb = Path.of(kUsbLogPath);
        if (Files.isDirectory(usb) && Files.isWritable(usb)) {
            noUsbLogAlert.set(false);
            return kUsbLogPath;
        }
        try {
            Files.createDirectories(Path.of(kOnboardLogPath));
        } catch (IOException ex) {
            DriverStation.reportError("Failed to create onboard log directory: " + ex.getMessage(), false);
        }
        noUsbLogAlert.set(true);
        return kOnboardLogPath;
    }
}
// comment to create pr