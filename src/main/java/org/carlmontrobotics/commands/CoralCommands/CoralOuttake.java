// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.CoralCommands;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralOuttake extends Command {
  /** Creates a new SetCoralOut. */
  private CoralEffector coralEffector;
  //private double timeOutTime;
  private double speed;
  //Timer timer;
  public CoralOuttake(CoralEffector coralEffector, double speed) {
    addRequirements(this.coralEffector = coralEffector);
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    //this.timeOutTime = time;
    //timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ///timer.reset();
    //timer.start();
    coralEffector.setMotorSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*    if (coralEffector.coralDetected()) {
      coralEffector.setMotorSpeed(CoralEffectorConstants.coralEffectorMotorOutputSpeed);
      timer.reset();
      timer.start();
    }
    else */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEffector.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
