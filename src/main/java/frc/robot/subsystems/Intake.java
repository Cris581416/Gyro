// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.CopyOnWriteArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  WPI_TalonFX intake;

  Solenoid cylinder;

  public static double SPEED = 0.0;

  public Intake() {
    intake = new WPI_TalonFX(IntakeConstants.intake);

    cylinder = new Solenoid(IntakeConstants.cylinder);

    setCylinder(true);
  }



  public void set(double speed){
    double limit = 0.5;

    if(Math.abs(speed) > limit){speed = limit * Math.signum(speed);}

    intake.set(speed);
  }


  public void setCylinder(boolean mode){

    cylinder.set(mode);

  }


  public void stop(){
    set(0.0);
  }

  

  public double getCurrentDraw(){

    TabData data = Shuphlebord.powerData;

    data.updateEntry("Intake", intake.getSupplyCurrent());

    return intake.getSupplyCurrent();

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getCurrentDraw();
  }
}
