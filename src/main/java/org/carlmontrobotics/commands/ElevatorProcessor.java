package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;

public class ElevatorProcessor extends Command {
    private final Elevator elevator;
    public ElevatorProcessor(Elevator elevator) {
        addRequirements(this.elevator = elevator);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setGoal(ElevatorPos.PROCESSOR);
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
