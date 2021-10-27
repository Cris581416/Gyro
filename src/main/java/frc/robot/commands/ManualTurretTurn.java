// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class ManualTurretTurn extends CommandBase {
  /** Creates a new ManualTurretTurn. */

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Turret turret;

  public ManualTurretTurn(Turret m_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);
    turret = m_turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double turnSpeed = RobotContainer.mechController.getX(Hand.kRight);

    turret.set(turnSpeed);

    RobotContainer.telemetry.turret.updateEntry("Position", turret.getPosition());

    System.out.println("Turret Position: " + turret.getPosition());

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
