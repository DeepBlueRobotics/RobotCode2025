package org.carlmontrobotics.commands;
import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;



public class OuttakeAlgae extends Command {
  AlgaeEffector algae;
  private final Timer timer = new Timer(); 
  public OuttakeAlgae(AlgaeEffector algaeEffector) {
    addRequirements(this.algae = algaeEffector);
  }

  @Override
  public void initialize() {
    //algae.setArmPosition(ARM_INTAKE_ANGLE);
    timer.reset();
    timer.start();

    
    algae.setPincherRPM(OUTTAKE_PINCHER_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // if (Math.abs(Algae.getArmPos()-Constants.AlgaeEffectorc.armRestingAngleWhileIntakeAlgae) <= Units.degreestoradian(5)) {
    //   Algae.setTopRPM(-1 * Constants.AlgaeEffectorc.intakeTopRPM);
    //   Algae.setBottomRPM(-1 * Constants.AlgaeEffectorc.intakeBottomRPM);
    //   Algae.setPincherRPM(-1 * Constants.AlgaeEffectorc.intakePincherRPM);
    // }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.stopPincherMotor();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {//FIXME DO THIS
    return timer.get()>3; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}