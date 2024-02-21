// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LClimb extends SubsystemBase {

  private final CANSparkMax m_lClimbMotor = new CANSparkMax(Constants.kRio_CAN_Climb_Left_Motor, MotorType.kBrushless);
  
  double climbSpeed = 2;
  double climbmax = 1000;
  double climbmin = 0;
  

  /** Creates a new Climb. */
  public LClimb() {
    m_lClimbMotor.getEncoder().setPosition(0);
    m_lClimbMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Left Climb Motor Pos", m_lClimbMotor.getEncoder().getPosition());

    if (m_lClimbMotor.getEncoder().getPosition() >= 250 && m_lClimbMotor.getEncoder().getVelocity() > 0) {
    m_lClimbMotor.set(0);
    }

    if (m_lClimbMotor.getEncoder().getPosition() <= 0 && m_lClimbMotor.getEncoder().getVelocity() < 0) {
    m_lClimbMotor.set(0);
    }


  }

  public void lClimbUp() {
    climbSpeed = climbSpeed;
    m_lClimbMotor.set(climbSpeed);
  }

  public void lClimbDown() {
    m_lClimbMotor.set(-climbSpeed);

  }

  public void lClimbStop() {
    m_lClimbMotor.set(0);
  }

}
