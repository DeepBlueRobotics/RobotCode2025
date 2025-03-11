package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class ShootAlgae extends Command {
  AlgaeEffector algae;
  private final Timer timer = new Timer(); 
  public ShootAlgae(AlgaeEffector algae) {
    addRequirements(this.algae = algae);
  }

  @Override
  public void initialize() {
    //algae.setArmPosition(Constants.AlgaeEffectorc.ARM_SHOOT_ANGLE);
    timer.reset();
    timer.start();

    algae.setTopRPM(Constants.AlgaeEffectorc.SHOOT_TOP_RPM);
    algae.setBottomRPM(Constants.AlgaeEffectorc.SHOOT_BOTTOM_RPM);
    algae.setPincherRPM(Constants.AlgaeEffectorc.SHOOT_PINCHER_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.stopMotors();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 3; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}