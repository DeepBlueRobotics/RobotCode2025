// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.DriveCommands;

import org.carlmontrobotics.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToNearest60 extends Command {
    
    private final RotateToFieldRelativeAngle rotateCommand;
    private final Drivetrain drivetrain;
    
    public RotateToNearest60(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        Rotation2d targetAngle = getNearest60(drivetrain.getHeading());
        this.rotateCommand = new RotateToFieldRelativeAngle(targetAngle, drivetrain);
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        rotateCommand.initialize();
    }
    
    @Override
    public void execute() {
        rotateCommand.execute();
    }
    
    @Override
    public boolean isFinished() {
        return rotateCommand.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        rotateCommand.end(interrupted);
    }
    
    private Rotation2d getNearest60(double currentAngle) {
        double modAngle = (Math.round(currentAngle / 60.0) * 60) % 360;
        if (modAngle > 180) modAngle -= 360; // Keep angle within -180 to 180
        return Rotation2d.fromDegrees(modAngle);
    }
}
