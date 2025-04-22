package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;
import org.carlmontrobotics.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

import static org.carlmontrobotics.Constants.Elevatorc.*;

public class ElevatorToBottomLimitSwitch extends Command {
    private Elevator elevator;
    private double pos;
    public ElevatorToBottomLimitSwitch(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        if (elevator.getBottomLimitSwitch()) {
            elevator.zeroPosition();
        } else {
            elevator.setMasterEncoder(elevatorOffset);
        }
        }

    @Override
    public void end(boolean interrupted) {
        elevator.zeroPosition();
    }
    public boolean isFinished() {
        return elevator.getBottomLimitSwitch();
    }

}
