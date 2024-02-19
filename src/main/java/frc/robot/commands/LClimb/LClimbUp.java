// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class LClimbUp extends Command {
  /** Creates a new LCimbUp. */
  public LClimbUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(RobotContainer.m_lClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_lClimb.lClimbUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_lClimb.lClimbStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
