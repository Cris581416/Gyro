// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class AlignTurret extends CommandBase {
  /** Creates a new AlignTurret. */
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Turret turret;
  Pose2d robotPose;
  private double kP = 0.07;
  private double kI = 0.0;
  private double kD = 0.0007;
  private double turretPower = 0.0;
  private double setpoint = 0.0;
  private PIDController controller = new PIDController(kP, kI, kD);

  TabData turretData = Shuphlebord.turretData;

  public AlignTurret(Turret m_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);
    turret = m_turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // If this doesn't work, just do "change all occurrences" of turretData back to RobotContainer.telemetry.hood and delete line 26

    turretData.updateEntry("kP", kP);
    turretData.updateEntry("kI", kI);
    turretData.updateEntry("kD", kD);
    turretData.updateEntry("Position", turret.getPosition());
    turretData.updateEntry("Power", turretPower);
    turretData.updateEntry("State", Turret.STATE.name());

    turretData.updateEntry("Adjust X", 0.0);
    turretData.updateEntry("Adjust Y", 0.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    turret.getCurrentDraw();

    double adjustedkP = (double) turretData.getEntryData("kP").getDouble();
    double adjustedkI = (double) turretData.getEntryData("kI").getDouble();
    double adjustedkD = (double) turretData.getEntryData("kD").getDouble();

    double adjustedX = (double) turretData.getEntryData("Adjust X").getDouble();
    double adjustedY = (double) turretData.getEntryData("Adjust Y").getDouble();

    Translation2d testTarget = new Translation2d(adjustedX, adjustedY);
    
    //---------------------------------------------------------------------------
    if(Turret.STATE == Turret.States.ALIGNING) {

      if(kP != adjustedkP || kI != adjustedkI || kD != adjustedkD){
        kP = adjustedkP;
        kI = adjustedkI;
        kD = adjustedkD;

        controller.setPID(kP, kI, kD);
      }

      if(Limelight.hasTarget()){

        setpoint = turret.getPosition() + Limelight.getTx();

      } else{

        setpoint = turret.getValidPosition(turret.getAngleToGoal(RobotContainer.robotPose));

      }

      if(setpoint > turret.maxDegrees){
        setpoint = turret.maxDegrees;
      } else if(setpoint < turret.minDegrees){
        setpoint = turret.minDegrees;
      }

      turretData.updateEntry("Setpoint", setpoint);

      controller.setSetpoint(setpoint);

      turretPower = -controller.calculate(turret.getPosition());

      turretData.updateEntry("PID Output", turretPower);
      turretData.updateEntry("Error", controller.getPositionError());

      turret.set(turretPower);

    //---------------------------------------------------------------------------
    } else if(Turret.STATE == Turret.States.MANUAL) {

      double turnSpeed = -RobotContainer.mechController.getX(Hand.kRight);

      double pos = turret.getPosition();

      if(pos >= turret.maxDegrees && turnSpeed < 0.0){
        turnSpeed = 0.0;
      } else if(pos <= turret.minDegrees && turnSpeed > 0.0){
        turnSpeed = 0.0;
      }
      
      turret.set(turnSpeed);

    }

    turretData.updateEntry("Position", turret.getPosition());
    turretData.updateEntry("Power", turretPower);
    turretData.updateEntry("State", Turret.STATE.toString());
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
