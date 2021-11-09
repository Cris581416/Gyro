// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class SetShooter extends CommandBase {
  /** Creates a new Shoot. */

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  Shooter shooter;

  Timer kickerTimer = new Timer();

  private double kP = 0.08;
  private double kI = 0.0045;
  private double kD = 0.0065;
  private double shooterPower = 0.0;
  private double setpoint = 0;
  private PIDController controller = new PIDController(kP, kI, kD);

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
    kickerTimer.reset();

    shooterData.updateEntry("kP", kP);
    shooterData.updateEntry("kI", kI);
    shooterData.updateEntry("kD", kD);
    shooterData.updateEntry("Velocity", shooter.getVelocity());
    shooterData.updateEntry("Power", shooterPower);
    shooterData.updateEntry("State", Shooter.STATE.name());
    shooterData.updateEntry("Setpoint", -4400.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.getCurrentDraw();

    double adjustedkP = (double) shooterData.getEntryData("kP").getDouble();
    double adjustedkI = (double) shooterData.getEntryData("kI").getDouble();
    double adjustedkD = (double) shooterData.getEntryData("kD").getDouble();

    double adjustedSetpoint = (double) shooterData.getEntryData("AdjSetpoint").getDouble();

    boolean pressed = RobotContainer.driveController.getTriggerAxis(Hand.kRight) != 0.0;

    if(Shooter.STATE == Shooter.States.REVVING && (pressed || Shooter.AUTO)){

      kickerTimer.start();

      lastPressed = true;

      Hopper.STATE = Hopper.States.FEEDING;

      if(kP != adjustedkP || kI != adjustedkI || kD != adjustedkD){
        kP = adjustedkP;
        kI = adjustedkI;
        kD = adjustedkD;

        controller.setPID(kP, kI, kD);
      }

      setpoint = adjustedSetpoint;

      controller.setSetpoint(setpoint);

      shooterData.updateEntry("Setpoint", setpoint);

      shooterPower = controller.calculate(shooter.getVelocity());

      shooterData.updateEntry("PID Output", shooterPower);
      shooterData.updateEntry("Error", controller.getPositionError());

      shooter.setSpeed(shooterPower);

      if(Math.abs(controller.getPositionError()) < 300.0){
        Shooter.STATE = Shooter.States.SHOOTING;
      }

    //---------------------------------------------------------------------------      
    } else if(Shooter.STATE == Shooter.States.SHOOTING && (pressed || Shooter.AUTO)){

      lastPressed = true;

      Hopper.STATE = Hopper.States.FEEDING;

      shooterData.updateEntry("Setpoint", setpoint);

      shooterPower = controller.calculate(shooter.getVelocity());

      shooterData.updateEntry("PID Output", shooterPower);
      shooterData.updateEntry("Error", controller.getPositionError());

      shooter.setSpeed(shooterPower);

      shooter.setKicker(1.0);

      kickerTimer.reset();

    //---------------------------------------------------------------------------
    } else if(Shooter.STATE == Shooter.States.STOPPED){

      shooter.setSpeed(0.0);
      shooter.setKicker(0.0);

      if(lastPressed){
        Hopper.STATE = Hopper.States.STOPPED;
      }

      if(pressed){
        Shooter.STATE = Shooter.States.REVVING;
      }

      lastPressed = false;

    } else{

      Shooter.STATE = Shooter.States.STOPPED;

    }


    shooterData.updateEntry("Velocity", shooter.getVelocity());
    shooterData.updateEntry("Power", shooterPower);
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
