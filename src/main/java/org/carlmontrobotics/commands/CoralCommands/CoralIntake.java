package org.carlmontrobotics.commands.CoralCommands;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;

import org.carlmontrobotics.Constants.CoralEffectorc;
import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntake extends Command {
    // public static double spin;
    // public static boolean fast2 = false;
    Timer timer = new Timer();
    // Timer timer2 = new Timer();
    // Timer coralInTimer = new Timer();
    public static double coralMotorPosition;
    private CoralEffector coralEffector;
    private boolean enableAutoIntake = true;

    public CoralIntake(CoralEffector coralEffector) {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(this.CoralEffector = CoralEffector);
        addRequirements(this.coralEffector = coralEffector);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (coralEffector.distanceSensorSeesCoral() && !coralEffector.limitSwitchSeesCoral()) {
            coralEffector.setMotorSpeed(INPUT_FAST_SPEED);
            coralMotorPosition = coralEffector.getEncoderPos(); // mark the position in rotations
            //coralEffector.coralIn = true;
            coralMotorPosition = coralEffector.getEncoderPos() + CoralEffectorc.CORAL_EFFECTOR_DISTANCE_SENSOR_OFFSET;
        }
        else if (coralEffector.distanceSensorSeesCoral() /*&& coralEffector.coralIn == false&*/ && coralEffector.limitSwitchSeesCoral()) {
            coralEffector.setMotorSpeed(INPUT_SLOW_SPEED); 
        }
        else {
            coralEffector.setMotorSpeed(0);
            coralEffector.setReferencePosition(coralMotorPosition);

        }
        enableAutoIntake = SmartDashboard.getBoolean("Enable Auto Coral Intake?", true);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralEffector.setMotorSpeed(0);
    }

    // Returns true when the command should end.a
    @Override
    public boolean isFinished() {
    return !enableAutoIntake; //|| !coralEffector.distanceSensorSeesCoral() && coralEffector.limitSwitchSeesCoral();
    }
}