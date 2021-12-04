// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class Retreat extends CommandBase {
  /** Creates a new Retreat. */
  Drivetrain drivetrain;

  Pose2d lastPose;
  double distance = 1.0;
  boolean finished = false;

  public Retreat(Drivetrain m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    drivetrain = m_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    lastPose = RobotContainer.robotPose;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(RobotContainer.robotPose.getX()) < Math.abs(lastPose.getX()) + distance){

      drivetrain.curvatureDrive(-0.2, 0.0);

    } else{

      finished = true;

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.curvatureDrive(0.0, 0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
