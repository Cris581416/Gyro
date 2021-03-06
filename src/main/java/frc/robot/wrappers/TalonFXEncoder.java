// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * 
 * A wrapper class used to access the TalonFX built in encoders in the same way as the Encoder class.
 * 
*/
public class TalonFXEncoder {

    WPI_TalonFX motor;
    WPI_TalonFX optionalMotor;

    boolean combine = false;

    double distancePerPulse = 1.0;

    public TalonFXEncoder(WPI_TalonFX source){
        motor = source;

        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
    
    public TalonFXEncoder(WPI_TalonFX source1, WPI_TalonFX source2){
        motor = source1;
        optionalMotor = source2;

        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        optionalMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        combine = true;
    }


    private double getRawPos(){
        double rawCounts = motor.getSelectedSensorPosition();

        if(combine){
            rawCounts += optionalMotor.getSelectedSensorPosition();
            rawCounts /= 2.0;
        }

        return rawCounts;
    }

    private double getRawVel(){
        double rawVel = motor.getSelectedSensorVelocity();

        if(combine){
            rawVel += optionalMotor.getSelectedSensorVelocity();
            rawVel /= 2.0;
        }

        return rawVel * 10.0;
    }


    /**
     * Sets the distance per pulse for the encoder, which is 1 by default.
     * Distance per pulse affects value returned by getDistance and getVelocity
     * @param factor The distance per pulse for the encoder
     */
    public void setDistancePerPulse(double factor){
        distancePerPulse = factor;
    }

    
    /**
     * 
     * @return Distance traveled in distance units
     */
    public double getDistance(){
        return distancePerPulse * getRawPos();
    }


    /**
     * 
     * @return Rotational velocity in distance units / second
     */
    public double getVelocity(){
        return distancePerPulse * getRawVel();
    }


    /**
     * Reset the encoders to 0.0
     */
    public void reset(){
        motor.setSelectedSensorPosition(0.0);

        if(combine){
            optionalMotor.setSelectedSensorPosition(0.0);
        }
    }
}
