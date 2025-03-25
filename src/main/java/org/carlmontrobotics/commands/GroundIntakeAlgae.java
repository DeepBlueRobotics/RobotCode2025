package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AlgaeEffector;
import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class GroundIntakeAlgae extends Command {
  private final AlgaeEffector algae;
  private final Timer timer = new Timer();
  
  public GroundIntakeAlgae(AlgaeEffector algae) {
    addRequirements(this.algae = algae);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    //algae.setArmTarget(ARM_INTAKE_ANGLE);
    //this command is strange in that it seems to handle the arm movement by itself, but I will leave it that way until someone changes it.

    
    algae.setPincherRPM(Constants.AlgaeEffectorc.INTAKE_PINCHER_RPM);
  }


  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //algae.setArmTarget(Constants.AlgaeEffectorc.ARM_RESTING_ANGLE_WHILE_INTAKE_ALGAE);
    algae.stopMotors();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    //distance sensor doesn't detect coral
    //TODO: make distance sensor stuff
    //TODO: add smartdashboard
    //TODO: Test different times
    return algae.isAlgaeIntaked() || timer.get()>5; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}