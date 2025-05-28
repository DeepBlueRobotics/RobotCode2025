package org.carlmontrobotics.commands.AlgaeCommands;
import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;
import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;


public class DealgaficationIntake extends Command {
  AlgaeEffector algae;
  private final Timer timer = new Timer(); 
  public DealgaficationIntake(AlgaeEffector algae) {
    addRequirements(this.algae = algae);
  }

  @Override
  public void initialize() {
    //algae.setArmPosition(ARM_DEALGAFYING_ANGLE);
    timer.reset();
    timer.start();

    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>5;
    //return false; //Simulator doesnt work propperly because limiswtich is non existant (only for simulator)
  }
}