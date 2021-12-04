// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  public enum States {REVVING, SHOOTING, STOPPED}

  public static States STATE = States.STOPPED;

  public static boolean AUTO = false;

  WPI_TalonFX shooterMotor1;
  WPI_TalonFX shooterMotor2;

  WPI_TalonFX kicker;

  public Shooter() {

    shooterMotor1 = new WPI_TalonFX(ShooterConstants.shooterMotor1);
    shooterMotor2 = new WPI_TalonFX(ShooterConstants.shooterMotor2);

    shooterMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooterMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    kicker = new WPI_TalonFX(ShooterConstants.kicker);
  }



  public void setSpeed(double speed){
    double limit = 1.0;

    if(Math.abs(speed) > limit){speed = limit * Math.signum(speed);}

    shooterMotor1.set(speed);
    shooterMotor2.set(-speed);
  }



  public void setKicker(double speed){
    kicker.set(speed);
  }



  public double getVelocity(){
    double tickVel1 = shooterMotor1.getSelectedSensorVelocity();
    double tickVel2 = -shooterMotor2.getSelectedSensorVelocity();

    double rpm1= tickVel1 * 600.0 * Constants.THROUGH_BORE_ENCODER_REVS_PER_TICK;
    double rpm2 = tickVel2 * 600.0 * Constants.THROUGH_BORE_ENCODER_REVS_PER_TICK;

    double velocity = (rpm1 + rpm2) / 2.0;

    return velocity;
  }



  public void stop(){
    setSpeed(0.0);
    setKicker(0.0);
  }



  public double[] getCurrentDraw(){

    TabData data = Shuphlebord.powerData;

    data.updateEntry("Shooter 1", shooterMotor1.getSupplyCurrent());
    data.updateEntry("Shooter 2", shooterMotor2.getSupplyCurrent());

    double[] powers = {shooterMotor1.getSupplyCurrent(), shooterMotor2.getSupplyCurrent()};

    return powers;

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
