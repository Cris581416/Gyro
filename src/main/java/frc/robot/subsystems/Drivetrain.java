// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  WPI_TalonFX topLeftMotor;
  WPI_TalonFX topRightMotor;
  WPI_TalonFX bottomLeftMotor;
  WPI_TalonFX bottomRightMotor;

  DifferentialDrive drivetrain;

  AHRS navX;
  public double gyroOffset = 0.0;
  public boolean firstRead = true;

  DifferentialDriveOdometry odometry;

  Pose2d pose;

  public Compressor compressor;

  public Drivetrain() {

    topLeftMotor = new WPI_TalonFX(Constants.topLeftMotor);
    topRightMotor = new WPI_TalonFX(Constants.topRightMotor);
    bottomLeftMotor = new WPI_TalonFX(Constants.bottomLeftMotor);
    bottomRightMotor = new WPI_TalonFX(Constants.bottomRightMotor);

    bottomLeftMotor.follow(topLeftMotor);
    bottomRightMotor.follow(topRightMotor);

    topLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    topRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    bottomLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    bottomRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    drivetrain = new DifferentialDrive(topLeftMotor, topRightMotor);

    navX = new AHRS();

    pose = new Pose2d();

    resetSensors();

    odometry = new DifferentialDriveOdometry(navX.getRotation2d());

    compressor = new Compressor();
  }

  public void arcadeDrive(double throttle, double turn){

    double throttleDirection = 1.0;
    double turnDirection = 1.0;

    if(throttle != 0.0){
      throttleDirection = throttle / Math.abs(throttle);

    } else{
      throttleDirection = 0.0;
    }

    if(turn != 0.0){
      turnDirection = turn / Math.abs(turn);

    } else{
      turnDirection = 0.0;
    }

    double intercept = 0.2;

    double slope = 0.5;

    throttle = slope * throttle + throttleDirection * intercept;
    turn = slope * turn + turnDirection * intercept;

    
    drivetrain.arcadeDrive(throttle, turn);
  }

  public void curvatureDrive(double throttle, double turn){
    drivetrain.curvatureDrive(throttle, turn, throttle == 0.0);
  }

  public void stop(){
    arcadeDrive(0.0, 0.0);
  }

  public double getTicks(String side){
    double averageTicks = 0.0;
    if(side.toLowerCase() == "left"){
      averageTicks = -topLeftMotor.getSelectedSensorPosition() - bottomLeftMotor.getSelectedSensorPosition();
      averageTicks /= 2;
    } else if(side.toLowerCase() == "right"){
      averageTicks = topRightMotor.getSelectedSensorPosition() + bottomRightMotor.getSelectedSensorPosition();
      averageTicks /= 2;
    }
    return averageTicks;
  }

  public Pose2d getPose(){
    return pose;
  }

  public void resetPose(){
    odometry.resetPosition(new Pose2d(), new Rotation2d());
    resetSensors();
  }

  public void resetSensors(){
    topLeftMotor.setSelectedSensorPosition(0.0);
    topRightMotor.setSelectedSensorPosition(0.0);
    bottomLeftMotor.setSelectedSensorPosition(0.0);
    bottomRightMotor.setSelectedSensorPosition(0.0);
    navX.calibrate();
    navX.reset();
  }

  public double readEncoders() {
    return topLeftMotor.getSelectedSensorPosition();
    //return topRightMotor.getSelectedSensorPosition();
    //return bottomLeftMotor.getSelectedSensorPosition();
    //return bottomRightMotor.getSelectedSensorPosition();
    //return navX.getRotation2d().getDegrees();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.

    double leftTicks = getTicks("left");
    double rightTicks = getTicks("right");

    double ticksPerRotation = 2048.0;
    double wheelCircumference = 6.0 * Math.PI;

    // Measurements taken by going forward 5 feet
    double motorRotations = 59653.66 / ticksPerRotation;
    double wheelRotations = 60.0 / wheelCircumference;
    
    // How much the motor rotated vs how much the wheel rotated
    double gearRatio = motorRotations / wheelRotations;

    // Rotations * wheel circumference = distance traveled. (dividing by gearRatio converts motor rotations to wheel rotations)
    double leftMeters = (leftTicks / ticksPerRotation) * wheelCircumference / gearRatio;
    double rightMeters = (rightTicks / ticksPerRotation) * wheelCircumference / gearRatio;
    //leftMeters *= 0.0254;
    //rightMeters *= 0.0254;

    /*
    double ticksPerInch = 997.06; //NEED TO FIND EXACT TICKS PER INCH
    double inchesPerMeter = 0.0254;

    double ticksPerMeter = ticksPerInch / inchesPerMeter;

    // ticks / ticks per revolution = number of revolutions,   number of revolutions * circumference = distance traveled
    double leftMeters = ((leftTicks / ticksPerMeter));
    double rightMeters = ((rightTicks / ticksPerMeter));
    */

    // Update the pose
    var gyroAngle = Rotation2d.fromDegrees(-navX.getAngle() - gyroOffset);

    pose = odometry.update(gyroAngle, leftMeters, rightMeters);

  }
}
