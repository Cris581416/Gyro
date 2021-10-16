// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  Shooter shooter;

  Timer kickerTimer = new Timer();

  double shootSpeed = -1.0;

  public Shoot(Shooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
    shooter = m_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kickerTimer.reset();
    kickerTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*Hopper.shoot = true;

    shooter.setSpeed(shootSpeed);

    if(kickerTimer.get() > 3.0){
      shooter.setKicker(-shootSpeed);
    }*/

    double turnSpeed = RobotContainer.driveController.getX(Hand.kRight);
    double hoodSpeed = RobotContainer.driveController.getY(Hand.kLeft);

    shooter.setTurret(turnSpeed);
    //shooter.setHood(hoodSpeed);
    shooter.setKicker(hoodSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Hopper.shoot = false;
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
