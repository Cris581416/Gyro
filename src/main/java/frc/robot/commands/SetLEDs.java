// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

public class SetLEDs extends CommandBase {

  LEDs leds;

  /** Creates a new SetLEDs. */
  public SetLEDs(LEDs m_leds) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_leds);
    leds = m_leds;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    leds.setLEDS(LEDs.confetti);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    leds.setLEDS(LEDs.confetti);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    leds.setLEDS(LEDs.red);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
