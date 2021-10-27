// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.TabData;
import frc.robot.subsystems.Hopper;

public class Revolve extends CommandBase {
  /** Creates a new Unjam. */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  Hopper hopper;

  boolean firstJam = false;
  boolean unjamming = false;

  TabData hopperData = RobotContainer.telemetry.hopper;

  Timer timer = new Timer();
  double time = 1.0;

  public Revolve(Hopper m_hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hopper);
    hopper = m_hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    hopperData.updateEntry("State", Hopper.STATE.name());

    //---------------------------------------------------------------------------
    if(Hopper.STATE == Hopper.States.STOPPED) {

      hopper.stop();
    
    //---------------------------------------------------------------------------
    } else if(Hopper.STATE == Hopper.States.INDEXING) {

      hopperData.updateEntry("Current Draw", hopper.getCurrentDraw());

      hopper.set(hopper.indexSpeed);

      if(Math.abs(hopper.getCurrentDraw()) > 4.0 && time >= 1.0){
        firstJam = true;
        Hopper.STATE = Hopper.States.REVERSED;
    
        timer.reset();
        timer.start();
    
        hopper.indexSpeed *= -1;

        hopper.set(hopper.indexSpeed);
      }
    //---------------------------------------------------------------------------

    } else if(Hopper.STATE == Hopper.States.REVERSED){

      hopper.set(hopper.indexSpeed);

      if((Math.abs(hopper.getCurrentDraw()) > 4.0 && time >= 1.0) || (time >= 4.0)){
        Hopper.STATE = Hopper.States.INDEXING;
    
        timer.reset();
        timer.start();
    
        hopper.indexSpeed *= -1;

        hopper.set(hopper.indexSpeed);
      }

    //---------------------------------------------------------------------------
    } else if(Hopper.STATE == Hopper.States.FEEDING) {

      hopper.set(hopper.shootSpeed);
    
    }
    

    //System.out.println("Timer timer: " + time);

    if(firstJam){
      time = timer.get();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
