// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class Spindex extends CommandBase {
  /** Creates a new Spindex. */

  Hopper hopper;

  public Spindex(Hopper m_hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hopper);
    hopper = m_hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.spin = !hopper.spin;

    if(hopper.spin){
      hopper.set(hopper.indexSpeed);
    } else{
      hopper.set(0.0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
