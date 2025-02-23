package org.carlmontrobotics.commands;
import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;


public class OuttakeAlgae extends Command {
  AlgaeEffector Algae;
  private final Timer timer = new Timer(); 
  public OuttakeAlgae(AlgaeEffector algaeEffector) {
    addRequirements(this.Algae = algaeEffector);
  }

  @Override
  public void initialize() {
    Algae.setArmAngle(Constants.AlgaeEffectorc.ARM_DEALGAFYING_ANGLE);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(Algae.getArmPos()-Constants.AlgaeEffectorc.ARM_DEALGAFYING_ANGLE) <= Units.degreesToRadians(5))
      Algae.setMotorSpeed(Constants.AlgaeEffectorc.DEALGAFY_TOP_RPM, Constants.AlgaeEffectorc.DEALGAFY_BOTTOM_RPM, Constants.AlgaeEffectorc.SHOOT_DEALGAFY_PINCHER_RPM); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Algae.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}