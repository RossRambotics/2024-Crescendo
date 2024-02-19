// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RClimb extends SubsystemBase {

 
  private final CANSparkMax m_rClimbMotor = new CANSparkMax(Constants.kRio_CAN_Climb_Right_Motor, MotorType.kBrushless);

  double climbSpeed = -0.4;
  double climbmax = 1000;
  double climbmin = 0;
  double mrClimbEncoder = m_rClimbMotor.getEncoder().getPosition();

  /** Creates a new Climb. */
  public RClimb() {
    m_rClimbMotor.getEncoder().setPosition(0);
    m_rClimbMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void rClimbUp() {
    m_rClimbMotor.set(-climbSpeed);
  }

  public void rClimbDown() {
    // if (mrClimbEncoder <= 10) {
    // m_rClimbMotor.set(0);
    // } else {
    // m_rClimbMotor.set(climbSpeed);
    // }

    m_rClimbMotor.set(climbSpeed);

  }

  public void rClimbStop() {
    m_rClimbMotor.set(0);
  }

  public class RClimbUp {
  }


}
