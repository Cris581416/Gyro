package frc.robot;

public class ShuffleboardData {

    public TabData drivetrain;
    public TabData hopper;
    public TabData hood;
    public TabData turret;
    public TabData shooter;

    public ShuffleboardData(){

        drivetrain = new TabData("Drivetrain");
        hopper = new TabData("Hopper");
        hood = new TabData("Hood");
        turret = new TabData("Turret");
        shooter = new TabData("Shooter");

        // Drivetrain Stuff

        drivetrain.getEntry("Limelight Distance");
        drivetrain.getEntry("Odometry Distance");

        // Hopper Stuff

        hopper.getEntry("Hopper State");

        // Hood Stuff

        hood.getEntry("kP");
        hood.getEntry("kI");
        hood.getEntry("kP");
        hood.getEntry("Position");
        hood.getEntry("Power");
        hood.getEntry("Hood State");


        // Turret Stuff

        turret.getEntry("kP");
        turret.getEntry("kI");
        turret.getEntry("kP");
        turret.getEntry("Position");
        turret.getEntry("Power");
        turret.getEntry("Hood State");

    }

}
