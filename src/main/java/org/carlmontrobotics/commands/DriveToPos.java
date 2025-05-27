// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import static org.carlmontrobotics.Constants.Drivetrainc.thetaPIDController;

import java.util.List;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.PhotonVisionCamera;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPos extends Command {
  /** Creates a new DriveToPos. */
  Drivetrain dt;
  PhotonVisionCamera camera;

  public DriveToPos(Drivetrain dt, PhotonVisionCamera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, camera);
    this.dt = dt;
    this.camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.getRobotPos3D(camera.getBestTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
