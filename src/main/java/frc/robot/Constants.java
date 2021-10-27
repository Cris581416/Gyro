// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Drivetrain
    public static final int topLeftMotor = 0;
    public static final int topRightMotor = 1;
    public static final int bottomLeftMotor = 2;
    public static final int bottomRightMotor = 3;

    // Turret
    public static final int turret = 4;
    public static final int turretEncoderA = 0;
    public static final int turretEncoderB = 1;

    // Hood
    public static final int hood = 5;
    public static final int hoodEncoderA = 2;
    public static final int hoodEncoderB = 3;

    // Shooter
    public static final int shooterMotor1 = 6;
    public static final int shooterMotor2 = 7;
    public static final int kicker = 9;

    // Intake
    public static final int intake = 8;
    
    // Hopper
    public static final int hopper = 10;

    

    // MISCELLANEOUS
    public static final double THROUGH_BORE_ENCODER_REVS_PER_TICK = 1.0 / 2048.0;



    /* ENCODER ORIENTATIONS

    + = positive, - = negative

       Mech    Orient. 1    Orient. 2
    |*********************************|
    | Hood   | Up:   -    | Down:  +  |
    |        |            |           |
    -----------------------------------
    | Turret | Left: -    | Right: +  |
    |        |            |           |
    -----------------------------------
    | LL     | LOC:  -    | ROC:  +   |
    |        |            |           |
    -----------------------------------
    */
}
