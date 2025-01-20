package org.carlmontrobotics.commands;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RunAlgae extends Command {

  public RunAlgae(AlgaeEffector algaeEffector, boolean inverted, boolean stop) {
    addRequirements(this.algaeEffector = algaeEffector);
    this.inverted = inverted;
    
  }

  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stop) {
      algaeEffector.RunAlgaeMotors(0,0);
    }
    else {
      if (inverted) {
        algaeEffector.RunAlgaeMotors(0.5, -0.5);
      }
      else {
          algaeEffector.RunAlgaeMotors(0.5, 0.5);
      }
    }
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