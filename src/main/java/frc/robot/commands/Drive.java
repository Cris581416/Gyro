// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Drivetrain drivetrain;

  TabData drivetrainData = Shuphlebord.drivetrainData;

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

    drivetrain.getCurrentDraw();

    if(drivetrain.firstRead){
      drivetrain.m_gyro.reset();
      drivetrain.firstRead = false;
    }

    drivetrain.m_odometry.update(drivetrain.m_gyro.getRotation2d(), -drivetrain.m_leftEncoder.getDistance(), drivetrain.m_rightEncoder.getDistance());
    drivetrainData.updateEntry("Pose X", drivetrain.getPose().getX());
    drivetrainData.updateEntry("Pose Y", drivetrain.getPose().getY());
    drivetrainData.updateEntry("Pose A", drivetrain.getPose().getRotation().getDegrees());
    drivetrainData.updateEntry("LEnc", drivetrain.m_leftEncoder.getDistance());
    drivetrainData.updateEntry("REnc", drivetrain.m_rightEncoder.getDistance());


    double throttle = 0.55 * RobotContainer.driveController.getY(Hand.kLeft);
    double turn = -0.55 * RobotContainer.driveController.getX(Hand.kRight);

    drivetrain.curvatureDrive(throttle, turn); 
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
