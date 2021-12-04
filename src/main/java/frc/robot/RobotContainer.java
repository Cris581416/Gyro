// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DTProperties;
import frc.robot.autocommands.AutoShoot;
import frc.robot.commands.AlignHood;
import frc.robot.commands.AlignTurret;
import frc.robot.commands.Revolve;
import frc.robot.commands.SetLEDs;
import frc.robot.commands.SetShooter;
import frc.robot.commands.Slurp;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();
  private final Turret m_turret = new Turret();
  private final Hood m_hood = new Hood();
  private final LEDs m_leds = new LEDs();

  public static Pose2d robotPose = new Pose2d();

  // CONTROL LAYOUTS
  public static final XboxController driveController = new XboxController(0);
  public static final XboxController mechController = new XboxController(1);

  // DRIVER BUTTONS
  public final JoystickButton intakeButton = new JoystickButton(driveController, Button.kBumperLeft.value);
  public final JoystickButton closePosButton = new JoystickButton(driveController, Button.kY.value);
  public final JoystickButton linePosButton = new JoystickButton(driveController, Button.kB.value);
  public final JoystickButton trenchPosButton = new JoystickButton(driveController, Button.kA.value);
  public final JoystickButton retreatButton = new JoystickButton(driveController, Button.kX.value);

  // MECH BUTTONS
  public final JoystickButton odoModeButton = new JoystickButton(mechController, Button.kY.value);
  public final JoystickButton llModeButton = new JoystickButton(mechController, Button.kB.value);
  public final JoystickButton manualModeButton = new JoystickButton(mechController, Button.kA.value);
  public final POVButton incHoodOffset = new POVButton(mechController, 0);
  public final POVButton decHoodOffset = new POVButton(mechController, 180);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new Drive(m_drivetrain));
    m_hopper.setDefaultCommand(new Revolve(m_hopper));
    m_turret.setDefaultCommand(new AlignTurret(m_turret));
    m_hood.setDefaultCommand(new AlignHood(m_hood));
    m_shooter.setDefaultCommand(new SetShooter(m_shooter));
    m_leds.setDefaultCommand(new SetLEDs(m_leds));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    intakeButton.whileHeld(new Slurp(m_intake));
    
    odoModeButton.whenPressed(() -> Turret.STATE = Turret.States.ODO_ALIGNING);
    llModeButton.whenPressed(() -> Turret.STATE = Turret.States.LL_ALIGNING);
    manualModeButton.whenPressed(() -> Turret.STATE = Turret.States.MANUAL);
    incHoodOffset.whenPressed(() -> Hood.OFFSET += Hood.INCREMENT);
    decHoodOffset.whenPressed(() -> Hood.OFFSET -= Hood.INCREMENT);
    closePosButton.whenPressed(() -> Hood.TARGET = Hood.Targets.CLOSE);
    linePosButton.whenPressed(() -> Hood.TARGET = Hood.Targets.LINE);
    trenchPosButton.whenPressed(() -> Hood.TARGET = Hood.Targets.TRENCH);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // CREATE PATHS
    RamseteCommand driveOffLine = getRamseteCommand(Robot.dOLDTrajectory);
    // RamseteCommand sPathToBalls = getRamseteCommand(Robot.sPTBTrajectory);
    // RamseteCommand intakeBalls = getRamseteCommand(Robot.iBTrajectory);
    RamseteCommand backToLine = getRamseteCommand(Robot.bTLTrajectory);
    RamseteCommand driveAndIntake = getRamseteCommand(Robot.dAITrajectory);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(Robot.dOLDTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return driveOffLine.andThen(() -> Hood.STATE = Hood.States.ALIGNING)
                      .andThen(new AutoShoot())
                      .andThen(() -> m_intake.extend())
                      .andThen(() -> m_intake.set(0.5))
                      .andThen(() -> Hopper.STATE = Hopper.States.INDEXING)
                      .andThen(driveAndIntake)
                      .andThen(backToLine)
                      .andThen(() -> m_intake.set(0.0))
                      .andThen(() -> m_intake.retract())
                      .andThen(new AutoShoot());
  }


  private RamseteCommand getRamseteCommand(Trajectory trajectory){

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DTProperties.ksVolts, DTProperties.kvVoltSecondsPerMeter, DTProperties.kaVoltSecondsSquaredPerMeter),
        DTProperties.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DTProperties.kPDriveVel, 0, 0),
        new PIDController(DTProperties.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::tankDriveVolts,
        m_drivetrain
    );

    return ramseteCommand;
  }
}