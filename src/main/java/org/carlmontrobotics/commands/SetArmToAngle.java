package org.carlmontrobotics.commands;
import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class setArmAngle extends Command {
  AlgaeEffector Algae;
  double angle;
  private final Timer timer = new Timer(); 
  public SetArmToAngle(AlgaeEffector algaeEffector, double angle ) {
    addRequirements(this.Algae = algaeEffector);
    this.angle = angle;
  }

  @Override
  public void initialize() {
     AlgaeEffector.setArmAngle(angle);
    
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Algae.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Algae.limitDetects() || timer.get() > 1; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}