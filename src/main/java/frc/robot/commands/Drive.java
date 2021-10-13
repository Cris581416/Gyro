// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Drivetrain drivetrain;

  public Drive(Drivetrain m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    drivetrain = m_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.arcadeDrive(RobotContainer.driveController.getY(Hand.kLeft),

    drivetrain.arcadeDrive(0.0, 0.0);

    //Pose2d pose = drivetrain.getPose();
    //double heading = pose.getRotation().getDegrees();

    //System.out.println(heading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
