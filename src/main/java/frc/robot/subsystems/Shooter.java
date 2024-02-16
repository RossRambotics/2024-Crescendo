// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {

  private final TalonFX m_topMotor = new TalonFX(Constants.kRio_CAN_Shooter_Top_Motor);
  private final TalonFX m_botMotor = new TalonFX(Constants.kRio_CAN_Shooter_Bottom_Motor);

  private GenericEntry m_ShootSpeakerBotVel = null;
  private GenericEntry m_ShootSpeakerTopVel = null;

  /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  /* Start at velocity 0, no feed forward, use slot 1 */
  private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
      false);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  /** Creates a new Indexer. */
  public Shooter() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    /*
     * Torque-based velocity does not require a feed forward, as torque will
     * accelerate the rotor up to the desired velocity by itself
     */
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_topMotor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("****** TOP SHOOTER MOTOR ERROR Could not apply configs, error code: " + status.toString());
    }

    for (int i = 0; i < 5; ++i) {
      status = m_botMotor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("****** BOTTOM SHOOTER MOTOR ERROR Could not apply configs, error code: " + status.toString());
    }

    m_topMotor.setInverted(true);
    m_botMotor.setInverted(false);

    m_ShootSpeakerTopVel = Shuffleboard.getTab("Shooter")
        .add("ShooterTop", -9.5).getEntry();

    m_ShootSpeakerBotVel = Shuffleboard.getTab("Shooter")
        .add("ShooterBottom", -9.5).getEntry();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooterTopRPS", m_topMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("shooterBottomRPS", m_botMotor.getVelocity().getValueAsDouble());
  }

  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(m_topMotor, 0.001);
    PhysicsSim.getInstance().addTalonFX(m_botMotor, 0.001);
  }

  public void stop() {
    m_topMotor.setControl(m_brake);
    m_botMotor.setControl(m_brake);

    m_topMotor.setControl(m_voltageVelocity.withVelocity(0));
    m_botMotor.setControl(m_voltageVelocity.withVelocity(0));
  }

  public void setShooterTopVel(double dRPS) {
    m_ShootSpeakerTopVel.setDouble(dRPS);
  }

  public void setShooterBottomVel(double dRPS) {
    m_ShootSpeakerBotVel.setDouble(dRPS);

  }

  public void shootReverse() {
    double speed = -.9;

    double desiredTopRPS = 9.5;
    double desiredBottomRPS = 9.5;

    m_topMotor.setControl(m_voltageVelocity.withVelocity(desiredTopRPS));
    m_botMotor.setControl(m_voltageVelocity.withVelocity(desiredBottomRPS));
  }

  public void start() {
    double desiredTopRPS = m_ShootSpeakerTopVel.getDouble(60);
    double desiredBotRPS = m_ShootSpeakerBotVel.getDouble(60);

    m_topMotor.setControl(m_voltageVelocity.withVelocity(desiredTopRPS));
    m_botMotor.setControl(m_voltageVelocity.withVelocity(desiredBotRPS));
  }

  public double getTopMotorSpeed() {

    return m_topMotor.getVelocity().getValueAsDouble();
  }

  public double getBottomMotorSpeed() {

    return m_botMotor.getVelocity().getValueAsDouble();
  }

  public boolean isShooterReady() {
    double topError = Math.abs(m_ShootSpeakerTopVel.getDouble(0.0) - getTopMotorSpeed());
    double botError = Math.abs(m_ShootSpeakerBotVel.getDouble(0.0) - getBottomMotorSpeed());
    // DataLogManager.log("%%%%%%%%%%%%%%%%% " + topError);

    // If the error is greater than % of requested then we are not ready
    double percent = 0.05;

    if (Math.abs(topError) > Math.abs(m_ShootSpeakerTopVel.getDouble(0.0)) * percent) {
      return false;
    }
    if (Math.abs(botError) > Math.abs(m_ShootSpeakerBotVel.getDouble(0.0)) * percent) {
      return false;
    }

    return true;
  }
}
