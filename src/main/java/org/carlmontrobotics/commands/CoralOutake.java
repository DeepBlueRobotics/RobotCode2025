// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralOutake extends Command {
  /** Creates a new SetCoralOut. */
  private CoralEffector coralEffector;
  Timer timer;
  public CoralOutake(CoralEffector coralEffector) {
    addRequirements(this.coralEffector = coralEffector);
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    coralEffector.setMotorSpeed(OUTPUT_SPEED);
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
    coralEffector.coralIn = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > OUTAKE_TIME_OUT || coralEffector.distanceSensorSeesCoral()==false;
  }
}
