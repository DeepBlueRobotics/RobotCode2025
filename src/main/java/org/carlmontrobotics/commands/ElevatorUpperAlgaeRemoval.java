package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;
import org.carlmontrobotics.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorUpperAlgaeRemoval extends Command {
    Elevator elevator;
    
    public ElevatorUpperAlgaeRemoval(Elevator elevator) {
        addRequirements(this.elevator = elevator);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setGoal(ElevatorPos.UPPERALGAE);
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
