package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeCoral extends Command {
  private final CoralEffector Coral;
  private final Timer timer = new Timer();
  public OuttakeCoral(CoralEffector Coral) {
    addRequirements(this.Coral = Coral);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Coral.setRPM(Constants.CoralEffectorc.outtakeRPM);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Coral.stopEffector();
  }

  @Override
  public boolean isFinished() {
    //distance sensor doesn't detect coral
    //TODO: make distance sensor stuff
    //TODO: add smartdashboard
    return (!Coral.coralDetects() && !Coral.limitDetects()) || timer.get() > 4;
  }
}