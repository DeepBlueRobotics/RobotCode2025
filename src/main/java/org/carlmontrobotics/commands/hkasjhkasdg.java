package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.CoralEffector;

import edu.wpi.first.wpilibj2.command.Command;

public class hkasjhkasdg extends Command {
    CoralEffector uk;
    public hkasjhkasdg(CoralEffector uk) {
        this.uk=uk;
    }
    @Override
    public void initialize() {
        uk.setMotorSpeed(0);
    }
}
