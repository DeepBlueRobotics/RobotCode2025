package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class MonkeyElevatorPos extends Command {
    private Elevator elevator;
    private double pos;
    public MonkeyElevatorPos(Elevator elevator, double pos) {
        this.pos = pos;
        addRequirements(this.elevator = elevator);
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        elevator.monkeyElevatorPos(pos);
    }
    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
    public boolean isFinished() {
        return elevator.monkeyElevatorPos(pos);
    }
}
