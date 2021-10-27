// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */

  // Hood States
  public static enum States {ALIGNING, MANUAL, RETRACTED}

  public static States STATE = States.MANUAL;

  WPI_TalonSRX hood;

  Encoder encoder;

  // rotations of hood per rotations of output shaft (encoder rotations)
  double gearRatio = 24.0 / 385.0;

  private double maxEncoderTicks = 3650.0;

  public double maxDegrees = ticksToDegrees(maxEncoderTicks);


  public Hood() {
    hood = new WPI_TalonSRX(Constants.hood);

    encoder = new Encoder(Constants.hoodEncoderA, Constants.hoodEncoderB);

    encoder.setReverseDirection(true);

    encoder.reset();
  }



  public void set(double speed){
    double limit = 0.35;

    if(Math.abs(speed) > limit){speed = limit * (speed / Math.abs(speed));}

    hood.set(speed);
  }



  /**
   @return Degrees of rotation
   */
  public double getPosition(){

    double ticks = encoder.get();
    
    double degrees = ticksToDegrees(ticks);

    return degrees;
  }



  /**
   @param setpoint Setpoint for the hood to reach
   @param tolerance How far the hood can be from the setpoint. Used in code as (tolerance / 100) (a percentage).
   @return Whether hood at setpoint or not
   */
  public boolean atSetpoint(double setpoint, double tolerance){
    double maxToleratedPos = setpoint + (setpoint * tolerance / 100.0);
    double minToleratedPos = setpoint - (setpoint * tolerance / 100.0);

    if(maxToleratedPos >= getPosition() && getPosition() >= minToleratedPos){
      return true;
    }

    return false;
  }



  /**
   @param distance Distance to use when calculating hood angle
   @return Angle to set hood to
   */
  public double calculateAngle(double distance){
    return 0.0;
  }


  
  public double getTicks(){
    return encoder.get();
  }



  // Utility Methods

  public void resetEncoder(){
    encoder.reset();
  }



  public double ticksToDegrees(double ticks){
    double degrees = ticks * Constants.THROUGH_BORE_ENCODER_REVS_PER_TICK * gearRatio * 360.0;

    return degrees;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
