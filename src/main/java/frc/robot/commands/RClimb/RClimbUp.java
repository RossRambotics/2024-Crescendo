// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RClimbUp extends Command {
  /** Creates a new RClimbUp. */
  public RClimbUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(RobotContainer.m_rClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_rClimb.rClimbUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_rClimb.rClimbStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
