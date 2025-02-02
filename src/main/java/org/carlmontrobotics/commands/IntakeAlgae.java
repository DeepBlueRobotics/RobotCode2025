package org.carlmontrobotics.commands;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAlgae extends Command {
  AlgaeEffector algaeEffector;
  boolean stop;
  int direction;

  public IntakeAlgae(AlgaeEffector algaeEffector) {
    addRequirements(this.algaeEffector = algaeEffector);
    this.direction = direction;
    this.stop = stop;    
  }

  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}