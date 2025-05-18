package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;
import org.carlmontrobotics.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorToPos extends Command {
    private Elevator elevator;
    private double pos;
    public ElevatorToPos(Elevator elevator, double pos) {
        this.elevator = elevator;
        this.pos = pos;
        addRequirements(elevator);
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        elevator.setGoal(pos);
    }
    public boolean isFinished() {
        return false;
    }

}
