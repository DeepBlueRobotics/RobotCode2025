package org.carlmontrobotics.commands;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RunAlgae extends Command {
  AlgaeEffector algaeEffector;
  boolean stop;
  int direction;
  double lowSpeed;
  double intakeSpeed;
  double mediumSpeed;
  double highSpeed;
  int speed;
  double currentSpeed;


  public RunAlgae(AlgaeEffector algaeEffector, int direction, boolean stop 
  ,int speed
  ) {
    addRequirements(this.algaeEffector = algaeEffector);
    this.direction = direction;
    this.stop = stop;
    this.speed = speed;
    lowSpeed = 0.8;
    mediumSpeed = 0.9;
    highSpeed = 1;
    intakeSpeed = 0.15;

  }

  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("Speed", speed);
    if (speed == 1) {
      currentSpeed = lowSpeed;
    }
    else if (speed == 2) {
      currentSpeed = mediumSpeed;
    }
    else {
      currentSpeed = highSpeed;
    }
    if (stop) {
      algaeEffector.RunAlgaeMotors(0,0);
    }
    else {
      if (direction == 1) {
        algaeEffector.RunAlgaeMotors(intakeSpeed, intakeSpeed);
      }
      else if (direction == 2) {
          algaeEffector.RunAlgaeMotors(-intakeSpeed, -intakeSpeed);
      }
      else if (direction == 3) {
        algaeEffector.RunAlgaeMotors(0.4, -0.4);
      }
      else if (direction == 4) {
        algaeEffector.RunAlgaeMotors(-0.37, 0.37);
      }
      else if (direction == 5) {
        algaeEffector.RunAlgaeMotors(-0.4,0.4);
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