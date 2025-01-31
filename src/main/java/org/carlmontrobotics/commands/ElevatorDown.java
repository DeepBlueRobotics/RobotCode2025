package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;
import org.carlmontrobotics.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDown extends Command {
    Elevator elevator;
    
    public ElevatorDown(Elevator elevator) {
        addRequirements(this.elevator = elevator);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setGoal(ElevatorPos.DOWN);
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
