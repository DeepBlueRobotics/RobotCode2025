package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;
import org.carlmontrobotics.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorToPos extends Command {
    Elevator elevator;
    ElevatorPos bruh;

    public ElevatorToPos(Elevator elevator, ElevatorPos pos) {
        this.elevator = elevator;
        bruh = pos;
        addRequirements(elevator);
    }
    @Override
    public void initialize() {
        elevator.setGoal(bruh);
    }

}
