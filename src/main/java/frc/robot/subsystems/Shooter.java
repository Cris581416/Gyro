// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  WPI_TalonFX shooterMotor1;
  WPI_TalonFX shooterMotor2;

  WPI_TalonSRX hood;

  WPI_TalonSRX turret;

  WPI_TalonFX kicker;

  public Shooter() {

    shooterMotor1 = new WPI_TalonFX(Constants.shooterMotor1);
    shooterMotor2 = new WPI_TalonFX(Constants.shooterMotor2);

    hood = new WPI_TalonSRX(Constants.hood);

    turret = new WPI_TalonSRX(Constants.turret);

    kicker = new WPI_TalonFX(Constants.kicker);
  }

  public void setSpeed(double speed){
    shooterMotor1.set(speed);
    shooterMotor2.set(-speed);
  }

  public void setHood(double speed){
    hood.set(speed);
  }

  public void setTurret(double speed){
    turret.set(speed);
  }

  public void setKicker(double speed){
    kicker.set(speed);
  }

  public void stop(){
    setSpeed(0.0);
    setTurret(0.0);
    setKicker(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
