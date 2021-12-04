// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.subsystems.Hood;

public class AlignHood extends CommandBase {
  /** Creates a new AlignHood. */
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Hood hood;
  private double kP = 0.1;
  private double kI = 0.0045;
  private double kD = 0.0025;
  private double hoodPower = 0.0;
  private double setpoint = 0;
  private PIDController controller = new PIDController(kP, kI, kD);

  TabData hoodData = Shuphlebord.hoodData;

  public AlignHood(Hood m_hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hood);
    hood = m_hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    hoodData.updateEntry("kP", kP);
    hoodData.updateEntry("kI", kI);
    hoodData.updateEntry("kD", kD);
    hoodData.updateEntry("Position", hood.getPosition());
    hoodData.updateEntry("Power", hoodPower);
    hoodData.updateEntry("State", Hood.STATE.name());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double adjustedkP = hoodData.getEntryData("kP").getDouble();
    double adjustedkI = hoodData.getEntryData("kI").getDouble();
    double adjustedkD = hoodData.getEntryData("kD").getDouble();    

    //---------------------------------------------------------------------------
    if(Hood.STATE == Hood.States.ALIGNING) {
      
      if(kP != adjustedkP || kI != adjustedkI || kD != adjustedkD){
        kP = adjustedkP;
        kI = adjustedkI;
        kD = adjustedkD;

        controller.setPID(kP, kI, kD);
      }

      setpoint = hood.calculateAngle() - Hood.OFFSET;

      hoodData.updateEntry("Setpoint", setpoint);

      if(setpoint > hood.maxDegrees){
        setpoint = hood.maxDegrees;
      }

      controller.setSetpoint(setpoint);

      double position = hood.getPosition();
      hoodPower = controller.calculate(position);

      hood.set(-hoodPower);

    //---------------------------------------------------------------------------
    } else if(Hood.STATE == Hood.States.MANUAL) {

      double hoodSpeed = RobotContainer.mechController.getY(Hand.kLeft);

      hood.set(hoodSpeed);

    //---------------------------------------------------------------------------
    } else if(Hood.STATE == Hood.States.RETRACTED) {

      hood.set(0.0);

    }

    hoodData.updateEntry("Position", hood.getPosition());
    hoodData.updateEntry("Power", hoodPower);
    hoodData.updateEntry("State", Hood.STATE.toString());
    hoodData.updateEntry("OFFSET", Hood.OFFSET);
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
