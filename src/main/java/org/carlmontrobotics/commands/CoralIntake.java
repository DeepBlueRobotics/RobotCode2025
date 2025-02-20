package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.CoralEffectorConstants;
import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;;

public class CoralIntake extends Command {
    public static double spin;
    public static boolean fast2 = false;
    Timer timer = new Timer();
    Timer timer2 = new Timer();
    Timer coralInTimer = new Timer();
    public static double coralMotorPosition;
    private CoralEffector coralEffector;

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
        // if (CoralEffector.distanceSensorSees) {
        //     coralIn = true;
        //     if (CoralEffector.limitSwitchSees) {
        //         CoralEffector.coralMotor.set(CoralEffectorConstants.coralEffectorMotorSlowSpeed);
        //         spin = CoralEffectorConstants.coralEffectorMotorSlowSpeed;
        //     } else {
        //         if (timer.get() < 0.15) {
        //             CoralEffector.coralMotor.set(CoralEffectorConstants.coralEffectorMotorFastSpeed);
        //             spin = CoralEffectorConstants.coralEffectorMotorFastSpeed;
        //         } else {
        //             CoralEffector.coralMotor.set(CoralEffectorConstants.coralEffectorMotorFastSpeed2);
        //             spin = CoralEffectorConstants.coralEffectorMotorFastSpeed2;
        //         }
        //     }
        // } else {
        //     CoralEffector.coralMotor.set(0);
        //     spin = 0;
        //     timer.restart();
        // }
        // CoralEffector.coralMotor.set(0.1);
        // spin = 10;
        SmartDashboard.putNumber("spin", spin);
        SmartDashboard.putNumber("timer", timer.get());
    // SmartDashboard.getBoolean("outakeGet", coralIn);r
        SmartDashboard.putBoolean("coral in", coralEffector.coralIsIn());
        if (CoralEffector.distanceSensorSees){
            coralEffector.setMotorSpeed(0.07);
            coralMotorPosition = coralEffector.getEncoderPos(); //rotations
            coralEffector.setCoralIn(true);
        }
        else if (coralEffector.coralIsIn()){
            coralEffector.setReferencePosition(coralMotorPosition - 0.1); //rotations
        }
        else {
            coralEffector.setMotorSpeed(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}