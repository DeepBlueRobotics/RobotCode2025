package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAlgae extends Command {
  private final AlgaeEffector Algae;
  private final Timer timer = new Timer();
  public IntakeAlgae(AlgaeEffector Algae) {
    addRequirements(this.Algae = Algae);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Algae.setTopRPM(Constants.AlgaeEffectorc.intakeTopRPM); 
    Algae.setBottomRPM(Constants.AlgaeEffectorc.intakeBottomRPM); 
    Algae.setPincherRPM(Constants.AlgaeEffectorc.intakePincherRPM); 
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Algae.stopMotors();
    //TODO: Test different times
  }

  @Override
  public boolean isFinished() {
    //distance sensor doesn't detect coral
    //TODO: make distance sensor stuff
    //TODO: add smartdashboard
    return Algae.limitDetects() || timer.get() > 1; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}