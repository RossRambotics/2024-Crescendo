// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    boolean isRedAlliance = false;
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        isRedAlliance = true;
      }
    }

    RobotContainer.m_shooter.setShooterTopVel(10);
    RobotContainer.m_shooter.setShooterBottomVel(10);

    if (isRedAlliance) {
      RobotContainer.m_tracking.setTargetID(5);
      RobotContainer.m_tracking.setTargetAngle(90);
    } else {
      RobotContainer.m_tracking.setTargetID(6);
      RobotContainer.m_tracking.setTargetAngle(90);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
