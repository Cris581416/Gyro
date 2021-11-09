// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {


    public final class DrivetrainConstants{

        public static final int topLeftMotor = 0;
        public static final int topRightMotor = 1;
        public static final int bottomLeftMotor = 2;
        public static final int bottomRightMotor = 3;

        public static final double gearRatio = 60.0 / 8.0;
        public static final double wheelCircumference = Math.PI * 4;

    }
    

    public final class TurretConstants{

        public static final int turret = 4;
        public static final int turretEncoderA = 0;
        public static final int turretEncoderB = 1;

    }


    public final class HoodConstants{

        public static final int hood = 5;
        public static final int hoodEncoderA = 2;
        public static final int hoodEncoderB = 3;

    }


    public final class ShooterConstants{

        public static final int shooterMotor1 = 6;
        public static final int shooterMotor2 = 7;
        public static final int kicker = 9;

    }


    public final class IntakeConstants{

        public static final int intake = 8;

        public static final int cylinder = 0;

    }
    

    public final class HopperConstants{

        public static final int hopper = 10;

    }


    public static final class DTProperties{

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.769;
        public static final double kvVoltSecondsPerMeter = 2.52;
        public static final double kaVoltSecondsSquaredPerMeter = 0.221;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 2.16;

        public static final double kTrackwidthMeters = 0.661234;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        private static final double inchesToMeters = 0.0254;
        public static final double kEncoderDistancePerPulse = inchesToMeters * DrivetrainConstants.wheelCircumference * THROUGH_BORE_ENCODER_REVS_PER_TICK / DrivetrainConstants.gearRatio;

    }


    public final class AutoConstants{

        public static final double kMaxSpeedMetersPerSecond = 0.5;//3.048;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;//3.0;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;

    }
    

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
    | LL     | LOC:  -    | ROC:   +  |
    |        |            |           |
    -----------------------------------
    | DT     | Left: +    | Right: -  |
    |        |            |           |
    -----------------------------------
    */
}
