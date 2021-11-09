package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public final class Goals {

    enum StartingPosition {LEFT, CENTER, RIGHT}


    // CHANGE ONLY THIS

    public static StartingPosition startingPos = StartingPosition.LEFT;

    
    public static Translation2d getGoalPosition(){

        Translation2d goalPos;

        if(startingPos == StartingPosition.LEFT){

            goalPos = new Translation2d(-3.175, 0.0);

        } else if(startingPos == StartingPosition.CENTER){

            goalPos = new Translation2d(-5.0, -5.0);

        } else {

            goalPos = new Translation2d(0.0, 0.0);
            
        }

        return goalPos;
    }
}
