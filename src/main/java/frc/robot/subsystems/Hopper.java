// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */

  WPI_TalonFX hopper;

  public static boolean shoot;
  public boolean spin;

  public double indexSpeed = 0.15;
  public double shootSpeed = -0.2;

  public Hopper() {
    hopper = new WPI_TalonFX(Constants.hopper);

    spin = false;
  }

  public void set(double speed){
    hopper.set(speed);
  }

  public double getCurrentDraw(){
    return hopper.getSupplyCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
