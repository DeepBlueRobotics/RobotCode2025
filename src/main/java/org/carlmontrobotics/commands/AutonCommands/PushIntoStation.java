// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AutonCommands;

import org.carlmontrobotics.subsystems.CoralEffector;
import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PushIntoStation extends Command {
  /** Creates a new PushIntoStation. */
  private Drivetrain drivetrain;
  private CoralEffector coralEffector;
  private boolean orginalFieldOrientation;
  /**
   * 
   * @param drivetrain
   * @param coralEffector
   */
  public PushIntoStation(Drivetrain drivetrain, CoralEffector coralEffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain = drivetrain, this.coralEffector = coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    orginalFieldOrientation = drivetrain.getFieldOriented();
    drivetrain.setFieldOriented(false);
    drivetrain.drive(-0.5,0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralEffector.coralSecured();
  }
}
