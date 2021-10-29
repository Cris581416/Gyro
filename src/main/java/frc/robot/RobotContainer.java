// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DTProperties;
import frc.robot.auto.AutoDrivetrain;
import frc.robot.commands.AlignHood;
import frc.robot.commands.AlignTurret;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Revolve;
import frc.robot.commands.SetShooter;
import frc.robot.commands.Slurp;
import frc.robot.commands.SwitchHopperMode;
import frc.robot.commands.SwitchTrackerModes;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final Drivetrain m_drivetrain = new Drivetrain();
  private final AutoDrivetrain m_robotDrive = new AutoDrivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();
  private final Turret m_turret = new Turret();
  private final Hood m_hood = new Hood();

  public static ShuffleboardData telemetry = new ShuffleboardData();

  public static final XboxController driveController = new XboxController(0);
  public static final XboxController mechController = new XboxController(1);

  public final JoystickButton switchHopperModeButton = new JoystickButton(mechController, Button.kBumperRight.value);
  public final JoystickButton intakeButton = new JoystickButton(mechController, Button.kBumperLeft.value);
  public final JoystickButton switchTrackerModesButton = new JoystickButton(mechController, Button.kB.value);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //m_drivetrain.setDefaultCommand(new Drive(m_drivetrain));
    m_hopper.setDefaultCommand(new Revolve(m_hopper));
    m_turret.setDefaultCommand(new AlignTurret(m_turret));
    m_hood.setDefaultCommand(new AlignHood(m_hood));
    m_shooter.setDefaultCommand(new SetShooter(m_shooter));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    intakeButton.whileHeld(new Slurp(m_intake));
    switchHopperModeButton.whenPressed(new SwitchHopperMode());
    switchTrackerModesButton.whenPressed(new SwitchTrackerModes());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DTProperties.ksVolts, DTProperties.kvVoltSecondsPerMeter,
            DTProperties.kaVoltSecondsSquaredPerMeter),
        DTProperties.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DTProperties.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DTProperties.ksVolts, DTProperties.kvVoltSecondsPerMeter,
            DTProperties.kaVoltSecondsSquaredPerMeter),
        DTProperties.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(DTProperties.kPDriveVel, 0, 0),
        new PIDController(DTProperties.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}
