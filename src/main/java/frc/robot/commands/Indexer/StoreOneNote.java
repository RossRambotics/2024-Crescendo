// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class StoreOneNote extends Command {
  private boolean m_isFinished = false;

  /** Creates a new storage. */
  public StoreOneNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(RobotContainer.m_indexer);
  }

  // Called

  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_indexer.isNoteBottom()) {
      RobotContainer.m_indexer.intake();
    } else {
      RobotContainer.m_indexer.stopBottom();
    }

    if (RobotContainer.m_indexer.isNoteTop()) {
      RobotContainer.m_indexer.Retract();
    } else {
      RobotContainer.m_indexer.stopTop();
    }

    m_isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override

  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
