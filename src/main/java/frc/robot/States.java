package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public final class States {

    enum StartingPosition {LEFT, CENTER, RIGHT}



    // CHANGE ONLY THIS

    public static StartingPosition startingPos = StartingPosition.CENTER;


    
    public static Translation2d getGoalPosition(){

        Translation2d goalPos;

        if(startingPos == StartingPosition.LEFT){

            goalPos = new Translation2d(0.0, 0.0);

        } else if(startingPos == StartingPosition.CENTER){

            goalPos = new Translation2d(-48.0, -24.0);

        } else {

            goalPos = new Translation2d(0.0, 0.0);
            
        }

        return goalPos;
    }
}
