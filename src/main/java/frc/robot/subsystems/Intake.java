// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  private boolean m_isExtended = true;
  private double m_sim_motor_speed = 0;
  private final CANSparkMax m_motor = new CANSparkMax(Constants.kRio_CAN_Intake_Motor, MotorType.kBrushless);

  /** Creates a new Indexer. */
  public Intake() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_motor.set(0);
    m_sim_motor_speed = 0;
  }

  public void intake() {
    double speed = 0.3;

    m_motor.set(speed);
    m_sim_motor_speed = speed;
  }

  public double getMotorSpeed() {
    if (Robot.isSimulation()) {
      return m_sim_motor_speed;
    }

    return m_motor.getEncoder().getVelocity();
  }

  public boolean isIntakeExtended() {
    return m_isExtended;
  }
}
