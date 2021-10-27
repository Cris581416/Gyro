// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.BufferedOutputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectOutputStream;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.TabData;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Drivetrain drivetrain;

  TabData drivetrainData = RobotContainer.telemetry.drivetrain;

  public Drive(Drivetrain m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    drivetrain = m_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(drivetrain.firstRead){
      drivetrain.gyroOffset = Drivetrain.getPose().getRotation().getDegrees();
      drivetrain.firstRead = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double throttle = 0.4 * RobotContainer.driveController.getY(Hand.kLeft);
    double turn = -0.4 * RobotContainer.driveController.getX(Hand.kRight);

    drivetrain.arcadeDrive(throttle, turn);

    Pose2d pose = Drivetrain.getPose();
    double poseY = pose.getX() + 0.0;
    double poseX = -pose.getY() + 0.0;
    double heading = pose.getRotation().getDegrees();

    drivetrainData.updateEntry("X", -pose.getY());
    drivetrainData.updateEntry("Y", pose.getX());
    drivetrainData.updateEntry("Heading", heading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
