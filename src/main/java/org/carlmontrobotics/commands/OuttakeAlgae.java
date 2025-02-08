package org.carlmontrobotics.commands;
import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeAlgae extends Command {
  AlgaeEffector Algae;
  private final Timer timer = new Timer(); 
  public OuttakeAlgae(AlgaeEffector algaeEffector) {
    addRequirements(this.Algae = algaeEffector);
  }

  @Override
  public void initialize() {
    Algae.setTopRPM(Constants.AlgaeEffectorc.outtakeTopRPM); 
    Algae.setBottomRPM(Constants.AlgaeEffectorc.outtakeBottomRPM); 
    Algae.setPincherRPM(Constants.AlgaeEffectorc.outtakePincherRPM); 
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Algae.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Algae.limitDetects() || timer.get() > 1; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}