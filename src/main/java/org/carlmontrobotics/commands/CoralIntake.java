package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntake extends Command {

    public CoralIntake() {
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements(this.CoralEffector = CoralEffector);    
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    if (CoralEffector.distanceSensorSees){
        if (CoralEffector.limitSwitchSees){
            CoralEffector.coralMotor.set(0.05);
        CoralEffector.coralMotor.set(0.1);
    }
    else{
        CoralEffector.coralMotor.set(0);
    }

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