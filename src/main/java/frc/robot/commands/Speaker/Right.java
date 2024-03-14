// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Speaker;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Right extends Command {
  /** Creates a new Right. */
  public Right() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("Speaker.Shoot.Right Requested.");
    RobotContainer.m_shooter.setShooterMessage("Speaker.Shoot.Right");
    var alliance = DriverStation.getAlliance();
    boolean isRedAlliance = false;
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        isRedAlliance = true;
      }
    }
    RobotContainer.m_shooter.setShooterTopVel(-55);
    RobotContainer.m_shooter.setShooterBottomVel(-45);

    RobotContainer.m_tracking.setTargetOffsetAdj(0.15);

    if (isRedAlliance) {
      RobotContainer.m_tracking.setTargetID(4);
      RobotContainer.m_tracking.setTargetAngle(-60 + 180);
    } else {
      RobotContainer.m_tracking.setTargetID(7);
      RobotContainer.m_tracking.setTargetAngle(-60);

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
