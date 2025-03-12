package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj2.command.Command;

public class StopCoralMotor extends Command {
    CoralEffector coralEffector;
    public StopCoralMotor(CoralEffector coralEffector) {
        this.coralEffector=coralEffector;
    }
    @Override
    public void initialize() {
        coralEffector.setMotorSpeed(0);
    }
}
