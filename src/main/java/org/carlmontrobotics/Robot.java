// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

//import org.carlmontrobotics.commands.CoralIntake;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  //private int autoFirstPri = 0;
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.print(m_robotContainer.getAutonomousCommand().toString());
    //autoFirstPri = Thread.currentThread().getPriority();

    if (m_autonomousCommand != null) {
      //Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.drivetrain.setDrivingIdleMode(true);
    m_robotContainer.elevator.setElevatorIdleMode(true);
    m_robotContainer.drivetrain.resetFieldOrientation();
    //m_robotContainer.drivetrain.resetFieldOrientationBackwards();
    //if (m_autonomousCommand != null) {
      //Thread.currentThread().setPriority(autoFirstPri);
      //m_autonomousCommand.cancel();
    //}
    CommandScheduler.getInstance().cancelAll();
    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    m_robotContainer.drivetrain.setDrivingIdleMode(false);
    m_robotContainer.elevator.setElevatorIdleMode(false);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
