// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  private boolean m_isExtended = true;
  private double m_sim_motor_speed = 0;
  private final CANSparkMax m_motor = new CANSparkMax(Constants.kRio_CAN_Intake_Motor, MotorType.kBrushless);
  private SparkPIDController m_PIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /** Creates a new Intake. */
  public Intake() {
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_PIDController = m_motor.getPIDController();

    // PID coefficients
    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_PIDController.setP(kP);
    m_PIDController.setI(kI);
    m_PIDController.setD(kD);
    m_PIDController.setIZone(kIz);
    m_PIDController.setFF(kFF);
    m_PIDController.setOutputRange(kMinOutput, kMaxOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    double speed = 0;
    m_PIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void intake() {
    double speed = 2 * 60;

    m_PIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void up() {
    m_isExtended = false;
  }

  public void down() {
    m_isExtended = true;
  }

  public void intakeReverse() {
    double speed = -1 * 60;

    m_PIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public double getMotorSpeed() {

    return m_motor.getEncoder().getVelocity();
  }

  public boolean isIntakeExtended() {
    return m_isExtended;
  }

  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(m_motor, 0.1f, 1000.0f);

  }
}
