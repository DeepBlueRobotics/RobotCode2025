package org.carlmontrobotics.commands.CoralCommands;

import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj2.command.Command;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;

import edu.wpi.first.wpilibj.Timer;

public class CoralSetSpeed extends Command{
    private CoralEffector coralEffector;
    double speed;
    double time;
    Timer timer = new Timer();
    public CoralSetSpeed(CoralEffector coralEffector, double speed, double time) {
        this.coralEffector = coralEffector;
        this.speed = speed;
        this.time = time;
    }
    @Override
    public void initialize() {
    timer.restart();
    }
    @Override
    public void execute() {
        coralEffector.setMotorSpeed(INTAKE_BACKWARDS_SPEED);
    } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEffector.setMotorSpeed(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= time;
  }
}
