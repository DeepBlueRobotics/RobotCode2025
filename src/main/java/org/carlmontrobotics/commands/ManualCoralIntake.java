// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import org.carlmontrobotics.subsystems.CoralEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualCoralIntake extends Command {
  /** Creates a new ManualCoralIntake. */
  private CoralEffector coralEffector;
  Timer timer = new Timer();
  public ManualCoralIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.coralEffector = coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralEffector.setMotorSpeed(INPUT_FAST_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > MANUAL_INTAKE_TIME_OUT;
  }
}
