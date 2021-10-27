// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class ManualHoodControl extends CommandBase {
  /** Creates a new ManualHoodControl. */

  Hood hood;

  public ManualHoodControl(Hood m_hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hood);
    hood = m_hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double hoodSpeed = RobotContainer.mechController.getY(Hand.kLeft);

    hood.set(hoodSpeed);

    RobotContainer.telemetry.hood.updateEntry("Position", hood.getPosition());

    System.out.println("Hood Position: " + hood.getPosition());

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
