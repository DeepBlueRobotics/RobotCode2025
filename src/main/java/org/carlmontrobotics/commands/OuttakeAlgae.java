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
    Algae.setArmPosition(Constants.AlgaeEffectorc.ARM_INTAKE_ANGLE);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armAtGoal(20))
      Algae.setTopRPM(Constants.AlgaeEffectorc.OUTTAKE_TOP_RPM);
      Algae.setBottomRPM(Constants.AlgaeEffectorc.OUTTAKE_BOTTOM_RPM);
      Algae.setPincherRPM(Constants.AlgaeEffectorc.OUTTAKE_PINCHER_RPM);
    
    // if (Math.abs(Algae.getArmPos()-Constants.AlgaeEffectorc.armRestingAngleWhileIntakeAlgae) <= Units.degreestoradian(5)) {
    //   Algae.setTopRPM(-1 * Constants.AlgaeEffectorc.intakeTopRPM);
    //   Algae.setBottomRPM(-1 * Constants.AlgaeEffectorc.intakeBottomRPM);
    //   Algae.setPincherRPM(-1 * Constants.AlgaeEffectorc.intakePincherRPM);
    // }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Algae.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {//FIXME DO THIS
    return timer.getFPGATimestamp()>3; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}