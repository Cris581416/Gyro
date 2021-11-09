// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Timer timer = new Timer();

  boolean finished = false;
  Shooter.States LAST_STATE;

  public AutoShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.stop();
    Shooter.STATE = Shooter.States.REVVING;
    LAST_STATE = Shooter.States.REVVING;
    Shooter.AUTO = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(LAST_STATE != Shooter.STATE){
      timer.reset();
      timer.start();
    }

    if(timer.get() > 2.0){
      finished = true;
    }

    LAST_STATE = Shooter.STATE;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Shooter.STATE = Shooter.States.STOPPED;
    Shooter.AUTO = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
