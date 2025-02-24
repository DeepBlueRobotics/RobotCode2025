package org.carlmontrobotics.commands;
import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;


public class Dealgafication extends Command {
  AlgaeEffector Algae;
  private final Timer timer = new Timer(); 
  public Dealgafication(AlgaeEffector algaeEffector) {
    addRequirements(this.Algae = algaeEffector);
  }

  @Override
  public void initialize() {
    Algae.setArmPosition(Constants.AlgaeEffectorc.ARM_DEALGAFYING_ANGLE);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armAtGoal(5))
      Algae.setTopRPM(Constants.AlgaeEffectorc.DEALGAFY_TOP_RPM); 
      Algae.setBottomRPM(Constants.AlgaeEffectorc.DEALGAFY_BOTTOM_RPM);
      Algae.setPincherRPM(Constants.AlgaeEffectorc.DEALGAFY_PINCHER_RPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Algae.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.getFPGATimestamp()>5 || Algae.isAlgaeIntaked();
    //return false; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}