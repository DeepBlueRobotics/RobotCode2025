package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SetArmToAngle extends Command {
  AlgaeEffector algaeEffector;
  double angle;
  private final Timer timer = new Timer(); 
  public SetArmToAngle(AlgaeEffector algaeEffector, double angle ) {
    addRequirements(this.algaeEffector = algaeEffector);
    this.angle = angle;
  }

  @Override
  public void initialize() {
     algaeEffector.setArmAngle(angle);
    timer.start();
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
    return false; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}