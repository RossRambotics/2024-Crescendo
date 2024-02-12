// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class GridSelector extends SubsystemBase {
    private Joystick m_bbox1 = new Joystick(1);
    private Joystick m_bbox2 = new Joystick(2);

    /** Creates a new GridSelector2. */
    public GridSelector() {

    }

    public void initialize() {
        Command cmd;

        Trigger btnShooterStart = new JoystickButton(m_bbox2, 2);
        cmd = new frc.robot.commands.Shooter.Start();
        btnShooterStart.onTrue(cmd);

    }

    @Override
    public void periodic() {
    }
}
