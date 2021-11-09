package frc.robot;

public class ShotData {


    static double[] angles = {1.0, 2.0};

    static double[] rspm = {};


    public static double getAngle(double distance){

        // Meters to feet
        distance = distance * 3.28084;

        int index = (int) ((double) Math.round(distance * 2.0) / 2.0);

        return angles[index];

    }

    public static double getRPM(double distance){

        // Meters to feet
        distance = distance * 3.28084;

        int index = (int) ((double) Math.round(distance * 2.0) / 2.0);

        return rspm[index];
        
    }

    public static double[] getData(double distance){

        double[] data = {getAngle(distance), getRPM(distance)};

        return data;

    }
}
