package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.carlmontrobotics.subsystems.CoralEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntakeManual extends Command {
  /** Creates a new CoralIntakeManual. */
  private CoralEffector coralEffector;
  public static double coralMotorPosition;
  Timer timer = new Timer();
  public CoralIntakeManual(CoralEffector coralEffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.coralEffector = coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralEffector.setMotorSpeed(INPUT_FAST_SPEED);
    coralMotorPosition = coralEffector.getEncoderPos(); //mark the position in rotations
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEffector.setReferencePosition(coralMotorPosition + CORAL_EFFECTOR_DISTANCE_SENSOR_OFFSET); //rotations
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > MANUAL_INTAKE_TIME_OUT;
  }
}
