// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  public static double tx;
  public static double ty;
  public static double ta;
  public static double thor;
  public static double tv;
  //public static 

  public static double x_initial = -15.3125; // Robot's initial x relative to starting target
  public static double y_initial = -115.75; // Robot's initial y relative to starting target
  public static double h_robot = 19.342962; //height of mount
  public static double h_target = 82.875; //height of target
  public static double a_limelight = 30.0; //38.83; //angle of limelight mount



  public Limelight() {
    reset();
  }



  static void reset(){
    tx = 0.0;
    ty = 0.0;
    ta = 0.0;
  }



  public static double getTx() {
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    return tx;
  }



  public static double getTy(){
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    return ty;
  }



  public static double getDistance(){

    double angleToTarget = a_limelight + getTy();

    double heightDiff = h_target - h_robot;

    double distance = heightDiff / Math.tan(Math.toRadians(angleToTarget));

    return distance;
  }



  public static Pose2d calculateDisplacement(double robotOrientation){
    Pose2d displacement;

    double distance = getDistance();

    double x_final = distance * Math.sin(robotOrientation); // Robot's final x position
    double y_final = distance * Math.cos(robotOrientation); // Robot's final y position

    double delta_x = x_final - x_initial;
    double delta_y = y_final - y_initial;

    displacement = new Pose2d(delta_x, delta_y, new Rotation2d(Math.toRadians(robotOrientation)));

    return displacement;
  }

  
  public static boolean hasTarget(){
    return tv == 1.0;
  }



  public static void update(){
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
    thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0.0);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    update();
  }
}
