// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Goals;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.Constants.HoodConstants;

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
    hood = new WPI_TalonSRX(HoodConstants.hood);

    encoder = new Encoder(HoodConstants.hoodEncoderA, HoodConstants.hoodEncoderB);

    encoder.setReverseDirection(true);

    encoder.reset();
  }



  public void set(double speed){
    double limit = 0.35;

    if(Math.abs(speed) > limit){speed = limit * Math.signum(speed);}

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



  public double calculateAngle(){
    return 0.0;
  }



  /**
   * 
   * @return The distance between the goal and the robot.
   */
  public double getDistance(){
    Translation2d goalPos = Goals.getGoalPosition();

    Pose2d pose = RobotContainer.robotPose;

    double deltaX = Math.abs(pose.getX() - goalPos.getX());
    double deltaY = Math.abs(pose.getY() - goalPos.getY());

    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    return distance;
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



  public double getCurrentDraw(){

    TabData data = Shuphlebord.powerData;

    data.updateEntry("Hood", hood.getSupplyCurrent());

    return hood.getSupplyCurrent();

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
