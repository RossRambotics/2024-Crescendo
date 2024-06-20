// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Indexer.StoreOneNote;
import frc.robot.sim.PhysicsSim;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final boolean UseLimelight = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);
    // m_robotContainer.m_intake.startCompresser();
    m_gcTimer.start();

    PathfindingCommand.warmupCommand().schedule();
  }

  Timer m_gcTimer = new Timer();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (UseLimelight) {
      var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

      Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

      if (lastResult.valid) {
        m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
      }
    }

    if (m_gcTimer.advanceIfElapsed(5.0)) {
      DataLogManager.log("%%%%%%%%%%% Before Garbage Collection: " + Runtime.getRuntime().freeMemory());
      System.gc();
      DataLogManager.log("%%%%%%%%%%% Ran Garbage Collection: " + Runtime.getRuntime().freeMemory());
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      DataLogManager.log("Auto Command: " + m_autonomousCommand.getName());
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationInit() {
    RobotContainer.m_shooter.simulationInit();
    RobotContainer.m_indexer.simulationInit();
    // RobotContainer.m_intake.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
    REVPhysicsSim.getInstance().run();
  }
}
