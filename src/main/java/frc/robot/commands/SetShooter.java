// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class SetShooter extends CommandBase {
  /** Creates a new Shoot. */

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  Shooter shooter;

  Timer shootTimer = new Timer();

  TabData shooterData = Shuphlebord.shooterData;

  boolean lastPressed = false;

  public SetShooter(Shooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
    shooter = m_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootTimer.reset();
    shooterData.updateEntry("Velocity", shooter.getVelocity());
    shooterData.updateEntry("State", Shooter.STATE.name());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean pressed = RobotContainer.driveController.getTriggerAxis(Hand.kRight) != 0.0;

    if(Shooter.STATE == Shooter.States.REVVING && (pressed || Shooter.AUTO)){

      shootTimer.start();

      lastPressed = true;

      Hopper.STATE = Hopper.States.FEEDING;
      Hood.STATE = Hood.States.ALIGNING;

      shooter.setSpeed(-1.0);

      shooterData.updateEntry("Velocity", shooter.getVelocity());

      if(shootTimer.get() > 2.5){
        Shooter.STATE = Shooter.States.SHOOTING;
      }

    //---------------------------------------------------------------------------      
    } else if(Shooter.STATE == Shooter.States.SHOOTING && (pressed || Shooter.AUTO)){

      lastPressed = true;

      Hopper.STATE = Hopper.States.FEEDING;

      shooter.setSpeed(-1.0);

      shooterData.updateEntry("Velocity", shooter.getVelocity());

      shooter.setKicker(1.0);

      shootTimer.reset();

    //---------------------------------------------------------------------------
    } else if(Shooter.STATE == Shooter.States.STOPPED){

      shooter.setSpeed(0.0);
      shooter.setKicker(0.0);

      shootTimer.reset();

      if(lastPressed){
        Hopper.STATE = Hopper.States.STOPPED;
        Hood.STATE = Hood.States.RETRACTED;
      }

      if(pressed){
        Shooter.STATE = Shooter.States.REVVING;
      }

      lastPressed = false;

    } else{

      Shooter.STATE = Shooter.States.STOPPED;

    }


    shooterData.updateEntry("Velocity", shooter.getVelocity());
    shooterData.updateEntry("State", Shooter.STATE.toString());
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
/* Ansh: Hi i'm gonna run some tests since you aren't here (6:06pm 11/13/2021)
...
man this is tiring*
...
nevermind I have no idea how this wiring works so -\_(;/)_/-*/