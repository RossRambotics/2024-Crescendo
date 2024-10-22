// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkPIDController;
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
  private final CANSparkMax m_topMotor = new CANSparkMax(Constants.kRio_CAN_Indexer_Top_Motor, MotorType.kBrushless);
  private final CANSparkMax m_bottomMotor = new CANSparkMax(Constants.kRio_CAN_Indexer_Bottom_Motor,
      MotorType.kBrushless);

  private SparkPIDController m_topPIDController;
  private SparkPIDController m_bottomPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

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
    m_topMotor.restoreFactoryDefaults();
    m_bottomMotor.restoreFactoryDefaults();

    // LiveWindow.enableTelemetry(m_TopSensorInput);
    // LiveWindow.enableTelemetry(m_MiddleSensorInput);
    // LiveWindow.enableTelemetry(m_BottomSensorInput);

    /**
     * In order to use PID functionality for a controller, a SparkPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_topPIDController = m_topMotor.getPIDController();
    m_bottomPIDController = m_bottomMotor.getPIDController();

    // PID coefficients
    kP = kP = 0.0001;
    ;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_topPIDController.setP(kP);
    m_topPIDController.setI(kI);
    m_topPIDController.setD(kD);
    m_topPIDController.setIZone(kIz);
    m_topPIDController.setFF(kFF);
    m_topPIDController.setOutputRange(kMinOutput, kMaxOutput);

    m_bottomPIDController.setP(kP);
    m_bottomPIDController.setI(kI);
    m_bottomPIDController.setD(kD);
    m_bottomPIDController.setIZone(kIz);
    m_bottomPIDController.setFF(kFF);
    m_bottomPIDController.setOutputRange(kMinOutput, kMaxOutput);

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
    SmartDashboard.putNumber("indexerTopRPM", m_topMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("indexerBottomRPM", m_bottomMotor.getEncoder().getVelocity());

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
    m_topPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
    m_bottomPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public void stopTop() {
    m_topPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public void stopBottom() {
    m_bottomPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public void shoot() {
    double setPointtop = -2 * 3500;
    double setPointbotom = -2 * 3500;

    if (RobotContainer.m_shooter.getTopMotorSpeed() > 0) {
      setPointtop = setPointtop * -1;
      setPointbotom = 0;
    }

    m_topPIDController.setReference(setPointtop, CANSparkMax.ControlType.kVelocity);
    m_bottomPIDController.setReference(setPointbotom, CANSparkMax.ControlType.kVelocity);
  }

  public void intake() {
    double setPoint = -2 * 2500;

    m_bottomPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void Retract() {
    double setPoint = 2 * 600;

    m_topPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void reverse() {
    double setPoint = 8 * 1000;

    m_topPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    m_bottomPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    // m_bottomPIDController.setReference(setPoint,
    // CANSparkMax.ControlType.kVelocity);
  }

  public double getTopMotorSpeed() {
    return m_topMotor.getEncoder().getVelocity();
  }

  public double getBottomMotorSpeed() {
    return m_bottomMotor.getEncoder().getVelocity();
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
    REVPhysicsSim.getInstance().addSparkMax(m_topMotor, 0.1f, 1000.0f);
    REVPhysicsSim.getInstance().addSparkMax(m_bottomMotor, 0.1f, 1000.0f);
  }

}
