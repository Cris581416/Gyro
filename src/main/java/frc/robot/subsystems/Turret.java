// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants;
import frc.robot.Goals;
import frc.robot.Shuphlebord;
import frc.robot.TabData;

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

    if(Math.abs(speed) > limit){speed = limit * Math.signum(speed);}

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
  

  public double getAngleToGoal(Pose2d robotPose){

    Translation2d goalPos = Goals.getGoalPosition();

    double deltaX = Math.abs(robotPose.getX() - goalPos.getX());
    double deltaY = Math.abs(robotPose.getY() - goalPos.getY());

    double goalError;

    if(deltaX == 0.0 && deltaY == 0.0){

      goalError = 0.0;

    } else{

      goalError = Math.atan(deltaY / deltaX);
    
    }

    double setpoint;

    if(robotPose.getY() >= goalPos.getY()){

      setpoint = Math.toRadians(robotPose.getRotation().getDegrees()) - goalError;

    } else{

      setpoint = Math.toRadians(robotPose.getRotation().getDegrees()) + goalError;

    }

    setpoint = Math.toDegrees(setpoint) % 360.0;

    return setpoint;
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

    double alternate = -Math.signum(degrees) * (360.0 - Math.abs(degrees));

    if(degrees > maxDegrees && alternate < minDegrees){
      return false;
    } else if(degrees < minDegrees && alternate > maxDegrees){
      return false;
    }
    return true;
  }


  
  public double getValidPosition(double degrees){

    double alternate = -Math.signum(degrees) * (360.0 - Math.abs(degrees));

    if(degrees > maxDegrees || degrees < minDegrees){
      return alternate;
    }

    return degrees;
  }



  public double getCurrentDraw(){

    TabData data = Shuphlebord.powerData;

    data.updateEntry("Turret", turret.getSupplyCurrent()); 

    return turret.getSupplyCurrent();

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}