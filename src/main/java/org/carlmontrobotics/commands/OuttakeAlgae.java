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
    Algae.setTopRPM(Constants.AlgaeEffectorc.outtakeTopRPM); 
    Algae.setBottomRPM(Constants.AlgaeEffectorc.outtakeBottomRPM); 
    Algae.setPincherRPM(Constants.AlgaeEffectorc.outtakePincherRPM); 
    Algae.setArmAngle(Constants.AlgaeEffectorc.armIntakeAngle);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(Algae.getArmPos()-Constants.AlgaeEffectorc.armRestingAngleWhileIntakeAlgae) <= Units.degreesToRadians(20))

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
  public boolean isFinished() {
    return false; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}