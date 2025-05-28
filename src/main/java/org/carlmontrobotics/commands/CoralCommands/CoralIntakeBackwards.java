package org.carlmontrobotics.commands.CoralCommands;

import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj2.command.Command;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;

public class CoralIntakeBackwards extends Command{
    private CoralEffector coralEffector;

    public CoralIntakeBackwards(CoralEffector coralEffector) {
        this.coralEffector = coralEffector;
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        coralEffector.setMotorSpeed(INTAKE_BACKWARDS_SPEED);
    } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEffector.setMotorSpeed(0);
    coralEffector.setReferencePosition(coralEffector.getEncoderPos());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
