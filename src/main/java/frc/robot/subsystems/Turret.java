// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  public static enum States {ALIGNING, MANUAL}

  public static States STATE = States.MANUAL;

  WPI_TalonSRX turret;

  Encoder encoder;

  // rotations of turret per rotations of output shaft (encoder rotations)
  double gearRatio = (24.0 / 150.0) * (25.0 / 34.0);

  public double maxDegrees = 140.0;//ticksToDegrees(maxEncoderTicks);
  public double minDegrees = -160.0;//ticksToDegrees(minEncoderTicks);

  public Turret() {
    turret = new WPI_TalonSRX(TurretConstants.turret);

    encoder = new Encoder(TurretConstants.turretEncoderA, TurretConstants.turretEncoderB);

    encoder.reset();
  }



  public void set(double speed) {
    double limit = 1.0;

    if(Math.abs(speed) > limit){speed = limit * speed / Math.abs(speed);}

    turret.set(speed);
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
   @param setpoint Setpoint for the turret to reach
   @param tolerance How far the turret can be from the setpoint. Used in code as (tolerance / 100) (a percentage).
   @return Whether turret at setpoint or not.
   */
  public boolean atSetpoint(double setpoint, double tolerance){
    double maxToleratedPos = setpoint + (setpoint * tolerance / 100.0);
    double minToleratedPos = setpoint - (setpoint * tolerance / 100.0);

    if(maxToleratedPos >= getPosition() && getPosition() >= minToleratedPos){
      return true;
    }

    return false;
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




  public boolean isValidPosition(double degrees){
    double alternate = 360.0 - degrees;

    if(degrees > maxDegrees && alternate < minDegrees){
      return false;
    } else if(degrees < minDegrees && alternate > maxDegrees){
      return false;
    }
    return true;
  }
  
  public double getValidPosition(double degrees){

    double alternate = 360.0 - degrees;

    if(degrees > maxDegrees || degrees < minDegrees){
      return alternate;
    }

    return degrees;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}