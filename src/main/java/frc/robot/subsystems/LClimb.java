// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LClimb extends SubsystemBase {

  private final CANSparkMax m_lClimbMotor = new CANSparkMax(Constants.kRio_CAN_Climb_Left_Motor, MotorType.kBrushless);

  double climbSpeed = 1;
  double currentClimbSpeed = 0;
  double climbmax = 1000;
  double climbmin = 0;

  private boolean m_isOverRide = false;

  private final Joystick m_bbox1 = new Joystick(1);

  /** Creates a new Climb. */
  public LClimb() {
    m_lClimbMotor.getEncoder().setPosition(5);
    m_lClimbMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Left Climb Motor Pos", m_lClimbMotor.getEncoder().getPosition());

    if (!m_isOverRide) {
      if (m_lClimbMotor.getEncoder().getPosition() >= 285 && m_bbox1.getRawButton(9) == true) {
        m_lClimbMotor.set(0);
      }

      if (m_lClimbMotor.getEncoder().getPosition() <= 0 && m_bbox1.getRawButton(1) == true) {
        m_lClimbMotor.set(0);
      }

    } else {
      m_lClimbMotor.set(currentClimbSpeed);
    }

  }

  public void lClimbUp() {
    currentClimbSpeed = climbSpeed;
    m_lClimbMotor.set(currentClimbSpeed);
  }

  public void lClimbDown() {
    currentClimbSpeed = -climbSpeed;
    m_lClimbMotor.set(currentClimbSpeed);

  }

  public void lClimbStop() {
    currentClimbSpeed = 0;
    m_lClimbMotor.set(currentClimbSpeed);
  }

  public Command getOverRideCommand() {
    return Commands.startEnd(() -> m_isOverRide = true, () -> m_isOverRide = false);
  }

}
