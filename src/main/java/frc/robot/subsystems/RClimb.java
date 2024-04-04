// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RClimb extends SubsystemBase {

  private final CANSparkMax m_rClimbMotor = new CANSparkMax(Constants.kRio_CAN_Climb_Right_Motor, MotorType.kBrushless);

  double climbSpeed = 1;
  double currentClimbSpeed = 0;
  double climbmax = 1000;
  double climbmin = 0;

  private boolean m_isOverRide = false;

  private final Joystick m_bbox2 = new Joystick(2);

  /** Creates a new Climb. */
  public RClimb() {
    m_rClimbMotor.setInverted(true);
    m_rClimbMotor.getEncoder().setPosition(5);
    m_rClimbMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Right Climb Motor Pos", m_rClimbMotor.getEncoder().getPosition());

    if (!m_isOverRide) {
      if (m_rClimbMotor.getEncoder().getPosition() >= 285 && m_bbox2.getRawAxis(1) == -1) {
        m_rClimbMotor.set(0);
      }

      if (m_rClimbMotor.getEncoder().getPosition() <= 0 && m_bbox2.getRawAxis(1) == 1) {
        m_rClimbMotor.set(0);

      }

    } else {
      m_rClimbMotor.set(currentClimbSpeed);
    }

  }

  public void rClimbUp() {
    currentClimbSpeed = climbSpeed;
    m_rClimbMotor.set(currentClimbSpeed);

  }

  public void rClimbDown() {
    currentClimbSpeed = -climbSpeed;
    m_rClimbMotor.set(currentClimbSpeed);

  }

  public void rClimbStop() {
    currentClimbSpeed = 0;
    m_rClimbMotor.set(currentClimbSpeed);

  }

  public Command getOverRideCommand() {
    return Commands.startEnd(() -> m_isOverRide = true, () -> m_isOverRide = false);
  }
}
