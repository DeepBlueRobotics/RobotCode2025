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
    Algae.setArmAngle(Constants.AlgaeEffectorc.ARM_INTAKE_ANGLE);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Algae.setArmAngle(Constants.AlgaeEffectorc.ARM_INTAKE_ANGLE)
    if (Algae.getArmPos() > Constants.AlgaeEffectorc.ARM_INTAKE_ANGLE) {
      Algae.setMotorSpeed(Constants.AlgaeEffectorc.INTAKE_TOP_RPM,Constants.AlgaeEffectorc.INTAKE_BOTTOM_RPM, Constants.AlgaeEffectorc.INTAKE_TOP_RPM);
      if (Algae.isAlgaeIntaked()) {
        Algae.setTopRPM(0);
        Algae.setBottomRPM(0);
        Algae.setArmAngle(Constants.AlgaeEffectorc.ARM_RESTING_ANGLE_WHILE_INTAKE_ALGAE)
      }
      

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