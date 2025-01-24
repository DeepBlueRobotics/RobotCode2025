package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;

import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RunCoral extends Command {
  // intake until sees game peice or 4sec has passed
  private final CoralEffector Coral;
  private double initSpeed = 2100;  

  public RunCoral(CoralEffector Coral) {
    addRequirements(this.Coral = Coral);
  }

  @Override
  public void initialize() {
    Coral.setRPMEffector(initSpeed); 
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    increaseSpeed = SmartDashboard.getNumber("Increase speed", 0);
    slowSpeed = SmartDashboard.getNumber("Slow intake speed", 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Coral.stopEffector();
  }

  @Override
  public boolean isFinished() {
    //distance sensor doesn't detect coral
    return Coral.outtakeDetectsNote();
  }
}