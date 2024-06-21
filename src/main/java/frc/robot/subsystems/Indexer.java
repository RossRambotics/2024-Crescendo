// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Indexer.StoreOneNote;

public class Indexer extends SubsystemBase {
  private final TalonFX m_topMotorR = new TalonFX(Constants.kRio_CAN_Indexer_Top_Motor);
  private final TalonFX m_topMotorL = new TalonFX(Constants.kRio_CAN_Indexer_Bottom_Motor);
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  // private SparkPIDController m_topPIDControllerR;
  // private SparkPIDController m_topPIDControllerL;
  // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public edu.wpi.first.wpilibj.AnalogInput m_TopSensorInput = new edu.wpi.first.wpilibj.AnalogInput(
      1);
  public edu.wpi.first.wpilibj.DigitalInput m_MiddleSensorInput = new edu.wpi.first.wpilibj.DigitalInput(
      0);
  public edu.wpi.first.wpilibj.AnalogInput m_BottomSensorInput = new edu.wpi.first.wpilibj.AnalogInput(
      3);

  public edu.wpi.first.wpilibj.DigitalInput m_TestInput = new edu.wpi.first.wpilibj.DigitalInput(
      3);

  GenericEntry m_TopSensor = null;
  GenericEntry m_MiddleSensor = null;
  GenericEntry m_BottomSensor = null;

  /** Creates a new Indexer. */
  public Indexer() {
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    // m_topMotorR.restoreFactoryDefaults();
    // m_topMotorL.restoreFactoryDefaults();

    // LiveWindow.enableTelemetry(m_TopSensorInput);
    // LiveWindow.enableTelemetry(m_MiddleSensorInput);
    // LiveWindow.enableTelemetry(m_BottomSensorInput);

    /**
     * In order to use PID functionality for a controller, a SparkPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    // m_topPIDControllerR = m_topMotorR.getPIDController();
    // m_topPIDControllerL = m_topMotorL.getPIDController();

    // PID coefficients
    // kP = kP = 0.0001;
    // ;
    // kI = 0;
    // kD = 0;
    // kIz = 0;
    // kFF = 0.000015;
    // kMaxOutput = 1;
    // kMinOutput = -1;
    // maxRPM = 5700;

    // set PID coefficients
    // m_topPIDControllerR.setP(kP);
    // m_topPIDControllerR.setI(kI);
    // m_topPIDControllerR.setD(kD);
    // m_topPIDControllerR.setIZone(kIz);
    // m_topPIDControllerR.setFF(kFF);
    // m_topPIDControllerR.setOutputRange(kMinOutput, kMaxOutput);

    // m_topPIDControllerL.setP(kP);
    // m_topPIDControllerL.setI(kI);
    // m_topPIDControllerL.setD(kD);
    // m_topPIDControllerL.setIZone(kIz);
    // m_topPIDControllerL.setFF(kFF);
    // m_topPIDControllerL.setOutputRange(kMinOutput, kMaxOutput);

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
    configs.Slot1.kP = 2.5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_topMotorR.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out
          .println("****** TOP Indexer Right MOTOR ERROR Could not apply configs, error code: " + status.toString());
    }

    for (int i = 0; i < 5; ++i) {
      status = m_topMotorL.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out
          .println("****** BOTTOM Indexer Left MOTOR ERROR Could not apply configs, error code: " + status.toString());
    }

    m_topMotorR.setInverted(true);
    m_topMotorL.setInverted(false);

    m_TopSensor = Shuffleboard.getTab("Indexer")
        .add("1-TopSensor", false).getEntry();
    m_MiddleSensor = Shuffleboard.getTab("Indexer")
        .add("2-MiddleSensor", false).getEntry();
    m_BottomSensor = Shuffleboard.getTab("Indexer")
        .add("3-BottomSensor", false).getEntry();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("indexerTopRPM",
    // m_topMotorR.getEncoder().getVelocity());
    // SmartDashboard.putNumber("indexerBottomRPM",
    // m_topMotorL.getEncoder().getVelocity());

    // TODO add read sensor values
    if (Robot.isReal()) {
      if (m_TopSensorInput.getValue() < 12.0) {
        m_TopSensor.setBoolean(true);
      } else {
        m_TopSensor.setBoolean(false);
      }
      if (!m_MiddleSensorInput.get()) {
        m_MiddleSensor.setBoolean(true);
      } else {
        m_MiddleSensor.setBoolean(false);
      }
      if (m_BottomSensorInput.getValue() < 12.0) {
        m_BottomSensor.setBoolean(true);
      } else {
        m_BottomSensor.setBoolean(false);
      }
    }

  }

  public void stop() {
    m_topMotorR.setControl(m_voltageVelocity.withVelocity(0));
    m_topMotorL.setControl(m_voltageVelocity.withVelocity(0));
  }

  public void stopTop() {
    m_topMotorR.setControl(m_voltageVelocity.withVelocity(0));
    m_topMotorL.setControl(m_voltageVelocity.withVelocity(0));
  }

  public void stopBottom() {
    m_topMotorR.setControl(m_voltageVelocity.withVelocity(0));
    m_topMotorL.setControl(m_voltageVelocity.withVelocity(0));
  }

  public void shoot() {
    double setPointtop = -2 * 7;
    double setPointbotom = -2 * 7;

    if (RobotContainer.m_shooter.getTopMotorSpeed() > 0) {
      setPointtop = setPointtop * -1;
      setPointbotom = 0;
    }
    m_topMotorR.setControl(m_voltageVelocity.withVelocity(setPointtop));
    m_topMotorL.setControl(m_voltageVelocity.withVelocity(setPointtop));

  }

  public void intake() {
    double setPoint = -2 * 5;

    m_topMotorR.setControl(m_voltageVelocity.withVelocity(setPoint));
    m_topMotorL.setControl(m_voltageVelocity.withVelocity(setPoint));
  }

  public void Retract() {
    double setPoint = 2 * 5;

    m_topMotorR.setControl(m_voltageVelocity.withVelocity(setPoint));
    m_topMotorL.setControl(m_voltageVelocity.withVelocity(setPoint));
  }

  public void reverse() {
    double setPoint = 8 * 5;

    m_topMotorR.setControl(m_voltageVelocity.withVelocity(setPoint));
    m_topMotorL.setControl(m_voltageVelocity.withVelocity(setPoint));
    // m_bottomPIDController.setReference(setPoint,
    // CANSparkMax.ControlType.kVelocity);
  }

  public double getTopMotorSpeed() {
    return m_topMotorR.getVelocity().getValueAsDouble();
  }

  public double getBottomMotorSpeed() {
    return m_topMotorL.getVelocity().getValueAsDouble();
  }

  public boolean isNoteTop() {
    return m_TopSensor.getBoolean(false);
  }

  public boolean isNoteMiddle() {
    return m_MiddleSensor.getBoolean(false);
  }

  public boolean isNoteBottom() {
    return m_BottomSensor.getBoolean(false);
  }

  public void simulationInit() {
    // REVPhysicsSim.getInstance().addSparkMax(m_topMotorR, 0.1f, 1000.0f);
    // REVPhysicsSim.getInstance().addSparkMax(m_topMotorL, 0.1f, 1000.0f);
  }

}
