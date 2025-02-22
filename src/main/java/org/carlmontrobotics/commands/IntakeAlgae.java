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
    Algae.setArmAngle(Constants.AlgaeEffectorc.armIntakeAngle);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Algae.getArmPos() > Constants.AlgaeEffectorc.armRestingAngleWhileIntakeAlgae) {
      
      Algae.setTopRPM(Constants.AlgaeEffectorc.intakeTopRPM);
      Algae.setBottomRPM(Constants.AlgaeEffectorc.intakeBottomRPM);
      Algae.setPincherRPM(Constants.AlgaeEffectorc.intakePincherRPM);
    }
  }

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
    return false; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}