// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RClimb.*;
import frc.robot.Constants;
import frc.robot.commands.LClimb.*;

/** Add your docs here. */
public class GridSelector extends SubsystemBase {
    private Joystick m_bbox1 = new Joystick(1);
    private Joystick m_bbox2 = new Joystick(2);
    PowerDistribution pDP = new PowerDistribution(Constants.pDP, ModuleType.kRev);

    /** Creates a new GridSelector2. */
    public GridSelector() {

    }

    public void initialize() {
        Command cmd;

        // Need to add climb commands
        Trigger joyRClimbUp = new Trigger(() -> m_bbox2.getRawAxis(1) == -1);
        cmd = new RClimbUp();
        joyRClimbUp.whileTrue(cmd);

        Trigger joyRClimbDown = new Trigger(() -> m_bbox2.getRawAxis(1) == 1);
        cmd = new RClimbDown();
        joyRClimbDown.whileTrue(cmd);

        Trigger joyLClimbUp = new Trigger(() -> m_bbox1.getRawAxis(1) == -1);
        cmd = new LClimbUp();
        joyLClimbUp.whileTrue(cmd);

        Trigger joyLClimbDown = new Trigger(() -> m_bbox1.getRawAxis(1) == 1);
        cmd = new LClimbDown();
        joyLClimbDown.whileTrue(cmd);

        // add Trap command
        // Trigger btnTrap = new JoystickButton(m_bbox2, 11);
        // cmd = new frc.robot.commands.Shooter.Start();
        // btnTrap.onTrue(cmd);

        // add Pass command
        // Trigger btnPass = new JoystickButton(m_bbox2, 11);
        // cmd = new frc.robot.commands.Shooter.Start();
        // btnPass.onTrue(cmd);

        Trigger btnAmp = new JoystickButton(m_bbox1, 11);
        cmd = new frc.robot.commands.Amp.Shoot();
        btnAmp.onTrue(cmd);

        Trigger btnIntakeOff = new JoystickButton(m_bbox2, 1);
        cmd = new frc.robot.commands.Intake.IntakeStop()
                .andThen(new frc.robot.commands.Intake.Up());
        btnIntakeOff.onTrue(cmd);

        Trigger btnIntakeIn = new JoystickButton(m_bbox2, 2);
        cmd = new frc.robot.commands.Intake.Up()
                .andThen(new frc.robot.commands.Intake.IntakeStart());
        btnIntakeIn.onTrue(cmd);

        Trigger btnIntakeReverse = new JoystickButton(m_bbox2, 2);
        cmd = new frc.robot.commands.Intake.IntakeReverse();
        btnIntakeReverse.whileTrue(cmd);

        Trigger btnIndexerShoot = new JoystickButton(m_bbox1, 1);
        cmd = new frc.robot.commands.Indexer.Shoot().repeatedly();
        btnIndexerShoot.whileTrue(cmd);

        // add indexer forward
        // Trigger btnIndexerForward = new JoystickButton(m_bbox1, 2);
        // cmd = new frc.robot.commands.Indexer.Shoot();
        // btnIndexerForward.onTrue(cmd);

        Trigger btnIndexerReverse = new JoystickButton(m_bbox1, 3);
        cmd = new frc.robot.commands.Indexer.Reverse().repeatedly();
        btnIndexerReverse.whileTrue(cmd);

        Trigger btnShooterReverse = new JoystickButton(m_bbox1, 4);
        cmd = new frc.robot.commands.Shooter.Reverse();
        btnShooterReverse.whileTrue(cmd);

        Trigger btnShooterStop = new JoystickButton(m_bbox1, 5);
        cmd = new frc.robot.commands.Shooter.Stop();
        btnShooterStop.whileTrue(cmd);

        Trigger btnShooterStart = new JoystickButton(m_bbox1, 6);
        cmd = new frc.robot.commands.Shooter.Start();
        btnShooterStart.onTrue(cmd);

        // Need to make a source command
        // Trigger btnSourceLeft = new JoystickButton(m_bbox2, 5);
        // cmd = new frc.robot.commands.Speaker.Left();
        // btnSourceLeft.onTrue(cmd);

        Trigger btnSourceMiddle = new JoystickButton(m_bbox1, 7);
        cmd = new frc.robot.commands.Source.Middle();
        btnSourceMiddle.onTrue(cmd);

        // Trigger btnSourceRight = new JoystickButton(m_bbox1, 8);
        // cmd = new frc.robot.commands.Speaker.Right();
        // btnSourceRight.onTrue(cmd);

        Trigger btnSpeakerLeft = new JoystickButton(m_bbox2, 6);
        cmd = new frc.robot.commands.Speaker.Left();
        btnSpeakerLeft.onTrue(cmd);

        Trigger btnSpeakerMiddle = new JoystickButton(m_bbox2, 7);
        cmd = new frc.robot.commands.Speaker.Middle();
        btnSpeakerMiddle.onTrue(cmd);

        Trigger btnSpeakerRight = new JoystickButton(m_bbox2, 8);
        cmd = new frc.robot.commands.Speaker.Right();
        btnSpeakerRight.onTrue(cmd);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Battery Volts", pDP.getVoltage());
    }
}
