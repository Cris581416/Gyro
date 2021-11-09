// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  // PATHS FOR AUTO
  static String dOLDir = "paths/driveoffline.wpilib.json";
  static String sPTBDir = "paths/spathtoballs.wpilib.json";
  static String iBDir = "paths/intakeballs.wpilib.json";
  static String bTLDir = "paths/backtoline.wpilib.json";
  static String dAIDir = "paths/driveandintake.wpilib.json";

  public static Trajectory dOLDTrajectory = new Trajectory();
  public static Trajectory sPTBTrajectory = new Trajectory();
  public static Trajectory iBTrajectory = new Trajectory();
  public static Trajectory bTLTrajectory = new Trajectory();
  public static Trajectory dAITrajectory = new Trajectory();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    try {
      Path dOLPath = Filesystem.getDeployDirectory().toPath().resolve(dOLDir);
      dOLDTrajectory = TrajectoryUtil.fromPathweaverJson(dOLPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + dOLDir, ex.getStackTrace());
    }

    try {
      Path sPTBPath = Filesystem.getDeployDirectory().toPath().resolve(sPTBDir);
      sPTBTrajectory = TrajectoryUtil.fromPathweaverJson(sPTBPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + sPTBDir, ex.getStackTrace());
    }

    try {
      Path iBPath = Filesystem.getDeployDirectory().toPath().resolve(iBDir);
      iBTrajectory = TrajectoryUtil.fromPathweaverJson(iBPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + iBDir, ex.getStackTrace());
    }

    try {
      Path bTLPath = Filesystem.getDeployDirectory().toPath().resolve(bTLDir);
      bTLTrajectory = TrajectoryUtil.fromPathweaverJson(bTLPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + bTLDir, ex.getStackTrace());
    }

    try {
      Path dAIPath = Filesystem.getDeployDirectory().toPath().resolve(dAIDir);
      dAITrajectory = TrajectoryUtil.fromPathweaverJson(dAIPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + dAIDir, ex.getStackTrace());
    }

    m_robotContainer = new RobotContainer();
    LiveWindow.disableAllTelemetry();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
