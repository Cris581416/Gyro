// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Revolve;
import frc.robot.commands.Shoot;
import frc.robot.commands.Slurp;
import frc.robot.commands.Spindex;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static final XboxController driveController = new XboxController(0);

  public final JoystickButton hopperButton = new JoystickButton(driveController, Button.kBumperRight.value);
  public final JoystickButton intakeButton = new JoystickButton(driveController, Button.kBumperLeft.value);
  public final JoystickButton shooterButton = new JoystickButton(driveController, Button.kY.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new Drive(m_drivetrain));
    //m_shooter.setDefaultCommand(new Shoot(m_shooter));
    m_hopper.setDefaultCommand(new Revolve(m_hopper));
    //m_intake.setDefaultCommand(new Slurp(m_intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    hopperButton.whenPressed(new Spindex(m_hopper));
    intakeButton.whileHeld(new Slurp(m_intake));
    shooterButton.whileHeld(new Shoot(m_shooter));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
