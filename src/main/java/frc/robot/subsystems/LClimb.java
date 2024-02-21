// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LClimb extends SubsystemBase {

  private final CANSparkMax m_lClimbMotor = new CANSparkMax(Constants.kRio_CAN_Climb_Left_Motor, MotorType.kBrushless);
  
  double climbSpeed = -
  0.4;
  double climbmax = 1000;
  double climbmin = 0;
  double mlClimbEncoder = m_lClimbMotor.getEncoder().getPosition();

  /** Creates a new Climb. */
  public LClimb() {
    m_lClimbMotor.getEncoder().setPosition(0);
  

    m_lClimbMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void lClimbUp() {
    m_lClimbMotor.set(climbSpeed);
  }

  public void lClimbDown() {
    // if (mlClimbEncoder <= 10) {
    // m_lClimbMotor.set(0);
    // } else {
    // m_lClimbMotor.set(-climbSpeed);
    // }

    m_lClimbMotor.set(-climbSpeed);
  }

  public void lClimbStop() {
    m_lClimbMotor.set(0);
  }

}
