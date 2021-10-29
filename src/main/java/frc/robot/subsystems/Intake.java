// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  WPI_TalonFX intake;



  public Intake() {
    intake = new WPI_TalonFX(IntakeConstants.intake);
  }



  public void set(double speed){
    double limit = 0.5;

    if(Math.abs(speed) > limit){speed = limit * speed / Math.abs(speed);}

    intake.set(speed);
  }



  public void stop(){
    set(0.0);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
