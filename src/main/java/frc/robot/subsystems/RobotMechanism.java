// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class RobotMechanism extends SubsystemBase {
  Mechanism2d mech = new Mechanism2d(4, 4);
  MechanismLigament2d intake = null;
  MechanismLigament2d indexerTL = null;
  MechanismLigament2d indexerTR = null;
  MechanismLigament2d indexerBL = null;
  MechanismLigament2d indexerBR = null;
  MechanismLigament2d shooterTop = null;
  MechanismLigament2d shooterBottom = null;
  MechanismLigament2d intakePistons = null;
  MechanismLigament2d topSensor = null;
  MechanismLigament2d middleSensor = null;
  MechanismLigament2d bottomSensor = null;

  double shooterTopAngle = 0;
  double shooterBottomAngle = 0;

  /** Creates a new RobotMechanism. */
  public RobotMechanism() {

    // Velocity

    // Position
    // MechanismLigament2d indexerTL = mech.
    // getRoot("indexerTL", 0.25, 0.5).
    // append(new MechanismLigament2d("robot", .2, 0, 0, new
    // Color8Bit(Color.kAliceBlue)));

    indexerTL = createMechWheel(mech, "indexerTL", .25, 2.0);
    indexerTR = createMechWheel(mech, "indexerTR", 1.25, 2.0);

    indexerBL = createMechWheel(mech, "indexerBL", .25, 1.25);
    indexerBR = createMechWheel(mech, "indexerBR", 1.25, 1.25);

    intake = createMechWheel(mech, "intake", .75, 0.5);
    intakePistons = createMechArrow(mech, "intakePistons", 1.5, 0.35);

    shooterTop = createMechWheel(mech, "shooterTop", .55, 3.5);
    shooterBottom = createMechWheel(mech, "shooterBottom", .85, 3);

    double d = 0.7; // bottom y for sensors
    double s = 0.8; // spacing for sensors
    topSensor = createMechSensor(mech, "topSensor", 2.0, d + (s * 2));
    middleSensor = createMechSensor(mech, "middleSensor", 2, d + (s * 1));
    bottomSensor = createMechSensor(mech, "bottomSensor", 2, d);
  }

  private int m_simFrame = 0;
  private boolean m_firstTime = true;

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      return;
    }
    if (m_firstTime) {
      m_firstTime = false;
      SmartDashboard.putData("Indexer.Shoot", new frc.robot.commands.Indexer.Shoot().andThen(new WaitCommand(10)));
      SmartDashboard.putData("Indexer.Reverse", new frc.robot.commands.Indexer.Reverse());
      SmartDashboard.putData("Indexer.Stop", new frc.robot.commands.Indexer.Stop());
      SmartDashboard.putData("Indexer.Intake", new frc.robot.commands.Indexer.Intake());
      SmartDashboard.putData("Indexer.Retract", new frc.robot.commands.Indexer.Retract());
      SmartDashboard.putData("Indexer.Storage", new frc.robot.commands.Indexer.StoreOneNote());
      SmartDashboard.putData("Shooter.Start", new frc.robot.commands.Shooter.Start());
      SmartDashboard.putData("Shooter.Stop", new frc.robot.commands.Shooter.Stop());
      SmartDashboard.putData("Shooter.Reverse", new frc.robot.commands.Shooter.Reverse());
      SmartDashboard.putData("Intake.IntakeStart", new frc.robot.commands.Intake.IntakeStart());
      SmartDashboard.putData("Intake.IntakeStop", new frc.robot.commands.Intake.IntakeStop());
      SmartDashboard.putData("Intake.IntakeReverse", new frc.robot.commands.Intake.IntakeReverse());
      SmartDashboard.putData("Intake.IntakeUp", new frc.robot.commands.Intake.Up());
      SmartDashboard.putData("Intake.DownStart",
          new frc.robot.commands.Intake.Down()
              .andThen(new frc.robot.commands.Intake.IntakeStart()));
      SmartDashboard.putData("Intake.StopUp",
          new frc.robot.commands.Intake.IntakeStop()
              .andThen(new frc.robot.commands.Intake.Up()));
      SmartDashboard.putData("Intake.IntakeStop", new frc.robot.commands.Intake.IntakeStop());
      SmartDashboard.putData("Speaker.Middle", new frc.robot.commands.Speaker.Middle());
      SmartDashboard.putData("Speaker.Right", new frc.robot.commands.Speaker.Right());
      SmartDashboard.putData("Speaker.Left", new frc.robot.commands.Speaker.Left());
      SmartDashboard.putData("Amp.Shoot", new frc.robot.commands.Amp.Shoot());

      Command c = new frc.robot.commands.Shooter.Start()
          .andThen(new frc.robot.commands.Indexer.Shoot())
          .andThen(new WaitCommand(2.0))
          .andThen(new frc.robot.commands.Shooter.Stop())
          .andThen(new frc.robot.commands.Indexer.Stop());

      SmartDashboard.putData("Robot.ShootSpeaker", c);

    }

    // This method will be called once per scheduler run
    m_simFrame += 5;
    if (m_simFrame > 10000) {
      m_simFrame = 0;
    }

    indexerTL.setAngle(RobotContainer.m_indexer.getTopMotorSpeed() * m_simFrame * 0.05);
    indexerTR.setAngle(-RobotContainer.m_indexer.getTopMotorSpeed() * m_simFrame * 0.05);

    indexerBL.setAngle(RobotContainer.m_indexer.getBottomMotorSpeed() * m_simFrame * 0.05);
    indexerBR.setAngle(-RobotContainer.m_indexer.getBottomMotorSpeed() * m_simFrame * 0.05);

    intake.setAngle(RobotContainer.m_intake.getMotorSpeed() * m_simFrame * 0.05);

    // shooter to falcons units is rotations per second, so convert to degrees
    shooterTopAngle += RobotContainer.m_shooter.getTopMotorSpeed() * 360 * 0.01;
    shooterBottomAngle += RobotContainer.m_shooter.getBottomMotorSpeed() * 360 * 0.01;
    if (shooterTopAngle > 360.0) {
      shooterTopAngle -= 360.0;
    }

    if (shooterBottomAngle > 360.0) {
      shooterBottomAngle -= 360.0;
    }

    shooterTop.setAngle(shooterTopAngle);
    shooterBottom.setAngle(-shooterBottomAngle);

    if (RobotContainer.m_intake.isIntakeExtended()) {
      intakePistons.setAngle(270);
      intakePistons.setColor(new Color8Bit(Color.kYellow));
    } else {
      intakePistons.setAngle(90);
      intakePistons.setColor(new Color8Bit(Color.kGreen));
    }

    if (RobotContainer.m_indexer.isNoteTop()) {
      topSensor.setColor(new Color8Bit(Color.kRed));
    } else {
      topSensor.setColor(new Color8Bit(Color.kWhite));
    }

    if (RobotContainer.m_indexer.isNoteMiddle()) {
      middleSensor.setColor(new Color8Bit(Color.kRed));
    } else {
      middleSensor.setColor(new Color8Bit(Color.kWhite));
    }

    if (RobotContainer.m_indexer.isNoteBottom()) {
      bottomSensor.setColor(new Color8Bit(Color.kRed));
    } else {
      bottomSensor.setColor(new Color8Bit(Color.kWhite));
    }

    SmartDashboard.putData("robot", mech); // Creates mech2d in SmartDashboard

  }

  static MechanismLigament2d createMechWheel(Mechanism2d mech, String name, double x, double y) {
    MechanismLigament2d wheel = mech.getRoot(name, x, y)
        .append(new MechanismLigament2d("robot", .2, 0, 0, new Color8Bit(Color.kAliceBlue)));

    MechanismLigament2d side1 = wheel
        .append(new MechanismLigament2d(name + ".side1", 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side2 = side1
        .append(new MechanismLigament2d(name + ".side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side3 = side2
        .append(new MechanismLigament2d(name + ".side3", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side4 = side3
        .append(new MechanismLigament2d(name + ".side4", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side5 = side4
        .append(new MechanismLigament2d(name + ".side5", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side6 = side5
        .append(new MechanismLigament2d(name + ".side6", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side7 = side6
        .append(new MechanismLigament2d(name + ".side7", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side8 = side7
        .append(new MechanismLigament2d(name + ".side8", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));

    return wheel;
  }

  static MechanismLigament2d createMechArrow(Mechanism2d mech, String name, double x, double y) {
    MechanismLigament2d arrow = mech.getRoot(name, x, y)
        .append(new MechanismLigament2d("robot", .3, 90, 3, new Color8Bit(Color.kGreen)));

    MechanismLigament2d side1 = arrow
        .append(new MechanismLigament2d(name + ".side1", 0.15307, 135, 3, new Color8Bit(Color.kWhite)));
    MechanismLigament2d side2 = arrow
        .append(new MechanismLigament2d(name + ".side2", 0.15307, -135, 3, new Color8Bit(Color.kWhite)));

    return arrow;
  }

  static MechanismLigament2d createMechSensor(Mechanism2d mech, String name, double x, double y) {
    MechanismLigament2d sensor = mech.getRoot(name, x, y)
        .append(new MechanismLigament2d("robot", .3, 90, 10, new Color8Bit(Color.kWhite)));

    return sensor;
  }

}
