// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.util.DistanceCalculator.DistanceCalculator;
import frc.util.DistanceCalculator.VisionMeasurement;

public class Tracking extends SubsystemBase {
  private GenericEntry m_TargetID = null; // current target to track
  private GenericEntry m_TargetAngle = null; // current target to track
  private GenericEntry m_isTargetIDFound = null; // is the current target found
  private GenericEntry m_isGamePieceFound = null; // does the game piece tracking camera see a gamepiece
  private GenericEntry m_TargetDistance = null; // distance to Current Target
  private GenericEntry m_TargetOffset = null; // robot-centric x distance to center of current target
  private GenericEntry m_GamePieceDistance = null; // distance to nearest gamepiece
  private GenericEntry m_GamePieceOffset = null; // robot-centric x distance to the center of the game piece

  private DistanceCalculator m_TargetDistanceCalculator = new DistanceCalculator();
  private DistanceCalculator m_NoteDistanceCalculator = new DistanceCalculator();

  private NetworkTable m_LL_Tracking = null;
  private NetworkTable m_LL_GamePiece = null;
  private double m_CurrentHeading = 0.0;
  private boolean m_isTargetTracking = false;
  private boolean m_isNoteTracking = false;

  /** Creates a new Tracking. */
  public Tracking() {
    // Establish April Tag Targeting LL
    m_LL_Tracking = NetworkTableInstance.getDefault().getTable("limelight-target");

    // Establish Game Piece tracking LL
    m_LL_GamePiece = NetworkTableInstance.getDefault().getTable("limelight-note");

    // Establish shuffleboard variables
    m_TargetID = Shuffleboard.getTab("Tracking")
        .add("TargetID", -1).getEntry();
    m_TargetAngle = Shuffleboard.getTab("Tracking")
        .add("TargetAngle", -1).getEntry();

    // Establish shuffleboard variables
    m_isTargetIDFound = Shuffleboard.getTab("Tracking")
        .add("isTargetIDFound", false).getEntry();
    m_isGamePieceFound = Shuffleboard.getTab("Tracking")
        .add("isGamePieceFound", false).getEntry();
    m_TargetDistance = Shuffleboard.getTab("Tracking")
        .add("TargetDistance", -1.0).getEntry();
    m_TargetOffset = Shuffleboard.getTab("Tracking")
        .add("TargetOffset", 0.0).getEntry();
    m_GamePieceDistance = Shuffleboard.getTab("Tracking")
        .add("GamePieceDistance", -1.0).getEntry();
    m_GamePieceOffset = Shuffleboard.getTab("Tracking")
        .add("GamePieceOffset", 0.0).getEntry();

    // set the bounds
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.0, 5.001));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(25.0, 0.0));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(100.0, 0.0));

    // Midas LL2 measurements
    // m_TargetDistanceCalculator.addSolution(new VisionMeasurement(1.7, 0.92));
    // m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.39, 2.02));
    // m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.16, 3.62));

    // Atlas LL2 measurements
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.082, 5.0));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.099, 4.5));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.0118, 4.0));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.148, 3.5));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.189, 3.0));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.250, 2.5));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.338, 2.0));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.481, 1.50));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.529, 1.40));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.583, 1.30));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.649, 1.20));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.727, 1.10));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(0.814, 1.0));
    m_TargetDistanceCalculator.addSolution(new VisionMeasurement(1.502, 0.20));

    // Note bounds setting (make anything closer 0.10m and anything further 4.0m)
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(-100, 0.10));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(-24, 0.10));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(20.0, 4.0));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(100.0, 4.0));

    // Atlas LL3 measurements
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(-20.04, 0.10));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(-18.24, 0.20));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(-16.27, 0.30));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(-12.72, 0.40));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(-9.15, 0.50));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(2.98, 1.0));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(8.51, 1.5));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(12.17, 2.0));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(14.47, 2.5));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(16.22, 3.0));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(17.26, 3.5));
    m_NoteDistanceCalculator.addSolution(new VisionMeasurement(18.59, 4.0));

    // DataLogManager.log("0.15: " +
    // m_TargetDistanceCalculator.compute(0.15).m_distance);
    // DataLogManager.log("0.16: " +
    // m_TargetDistanceCalculator.compute(0.16).m_distance);
    // DataLogManager.log("0.17: " +
    // m_TargetDistanceCalculator.compute(0.17).m_distance);
    // DataLogManager.log("0.38: " +
    // m_TargetDistanceCalculator.compute(0.38).m_distance);
    // DataLogManager.log("1.60: " +
    // m_TargetDistanceCalculator.compute(1.60).m_distance);
    // DataLogManager.log("1.70: " +
    // m_TargetDistanceCalculator.compute(1.70).m_distance);
    // DataLogManager.log("1.80: " +
    // m_TargetDistanceCalculator.compute(1.80).m_distance);
    // DataLogManager.log("2.00: " +
    // m_TargetDistanceCalculator.compute(2.00).m_distance);
    DataLogManager.log("0.00: " +
        m_NoteDistanceCalculator.compute(0.00).m_distance);
  }

  public void setCurrentHeading(double degrees) {
    m_CurrentHeading = degrees;
  }

  @Override
  public void periodic() {
    double dv = 0.0;

    // This method will be called once per scheduler run

    // do we see the correct target
    dv = m_LL_Tracking.getEntry("tv").getDouble(0);
    if (dv > 0.0) {
      // saw an april tag... so check if it is the correct one
      double da = m_LL_Tracking.getEntry("tid").getDouble(-1.0);
      if (dv > 0 && da == m_TargetID.getDouble(-1.0)) {
        m_isTargetIDFound.setBoolean(true);
      } else {
        m_isTargetIDFound.setBoolean(false);
      }
    }

    // do we see a game piece
    dv = m_LL_GamePiece.getEntry("tv").getDouble(0);
    if (dv > 0) {
      m_isGamePieceFound.setBoolean(true);
    } else {
      m_isGamePieceFound.setBoolean(false);
    }

    // if we are tracking an april tag calculate distance and offset
    if (this.isTargetIDFound()) {

      this.calcTargetDistanceSpeaker();
    }

    // if we are tracking a game piece calculate distance and offset
    if (this.isGamePieceFound()) {

      this.calcGamePieceDistance();
    }

  }

  private void calcGamePieceDistance() {
    // tx Range is -/+29.8 degrees
    // ty Range is -/+24.85 degrees
    double tx = m_LL_GamePiece.getEntry("tx").getDouble(0);
    double ty = m_LL_GamePiece.getEntry("ty").getDouble(0);

    double distance = m_NoteDistanceCalculator.compute(ty).m_distance;
    double offset = distance * Math.tan(Math.toRadians(tx));

    // double distance = (400.546 * Math.tan(Math.toRadians(0.114604 * (ty +
    // 8.1186)))) + 938.939;
    // distance = distance / 1000;
    // double offset = distance * Math.tan(Math.toRadians(tx));

    // Since targeting camera is pointing forward these are NOT inverted
    m_GamePieceDistance.setDouble(distance);
    m_GamePieceOffset.setDouble(offset);
  }

  public boolean isNoteTracking() {
    return m_isNoteTracking;
  }

  public boolean isTargetTracking() {
    return m_isTargetTracking;
  }

  private void calcTargetDistanceAmp() {

    double tx = m_LL_Tracking.getEntry("tx").getDouble(0);
    double ty = m_LL_Tracking.getEntry("ty").getDouble(0);
    double hr = 65;
    double ha = 133.5 - hr;
    double distance = ha / Math.tan(Math.toRadians(ty + 45));

    double offset = distance * Math.tan(Math.toRadians(tx));

    m_TargetDistance.setDouble(-distance);
    m_TargetOffset.setDouble(-offset);
  }

  private void calcTargetDistanceSpeaker() {

    double tx = m_LL_Tracking.getEntry("tx").getDouble(0);
    double ty = m_LL_Tracking.getEntry("ty").getDouble(0);
    double hr = 65;
    double hs = 143.5 - hr;
    double distance = hs / Math.tan(Math.toRadians(ty + 45));

    double offset = distance * Math.tan(Math.toRadians(tx));

    m_TargetDistance.setDouble(-distance);
    m_TargetOffset.setDouble(-offset);
  }

  private void calcTargetDistance() {
    // tx Range is -/+29.8 degrees
    // ty Range is -/+24.85 degrees

    double tx = m_LL_Tracking.getEntry("tx").getDouble(0);
    double ta = m_LL_Tracking.getEntry("ta").getDouble(0);
    double distance = m_TargetDistanceCalculator.compute(ta).m_distance;
    double offset = distance * Math.tan(Math.toRadians(tx));

    // Since targeting camera is pointing backward these are inverted
    m_TargetDistance.setDouble(-distance);
    m_TargetOffset.setDouble(-offset);
  }

  public void setTargetID(int id) {
    m_TargetID.setDouble(id);
  }

  public boolean isTargetIDFound() {
    return m_isTargetIDFound.getBoolean(false);
  }

  public boolean isGamePieceFound() {
    return m_isGamePieceFound.getBoolean(false);
  }

  public boolean isOnTarget() {
    double angleError = Math.abs(this.getTargetAngle().getDegrees() - m_CurrentHeading);
    double distanceError = Math.abs(this.getTargetDistanceError());
    double offsetError = Math.abs(m_TargetOffset.getDouble(0.0));

    if (angleError > 5.0) {
      return false;
    }

    if (distanceError > 0.05) {
      return false;
    }

    if (offsetError > 0.05) {
      return false;
    }

    return true;
  }

  public double getTargetDistanceError() {
    // figure out the appropriate goal distance based on the target ID
    // the goal should be either 0 or negative
    // goals are negative because the robot needs to BACK up for targets
    int targetID = (int) m_TargetID.getDouble(-1);
    double goal = 0.0;
    switch (targetID) {
      case 7:
      case 4:
        if (m_TargetAngle.getDouble(0) == 0 || m_TargetAngle.getDouble(0) == 180) {
          goal = -0.9; // TODO tune this
        } else {
          goal = -1.5;
        }
        break;
      default:
        goal = 0.0;
    }

    return m_TargetDistance.getDouble(0.0) - goal;
  }

  public double getTarget_VelocityY_Adjusted(DoubleSupplier joystick_value) {

    // see if we should just use the joystick because we aren't on heading or too
    // far away, etc.
    double angleError = this.getTargetAngle().getDegrees() - m_CurrentHeading;

    DataLogManager.log(
        "Angle Error: " + angleError + " Found?: " + isTargetIDFound() + " Distance: " + m_TargetDistance.getDouble(3));

    if (Math.abs(angleError) > 5) {
      return joystick_value.getAsDouble();
    }
    if ((!isTargetIDFound())) {
      return joystick_value.getAsDouble();
    }

    if (Math.abs(m_TargetDistance.getDouble(5)) > 5.0) {
      return joystick_value.getAsDouble();
    }

    double vY = this.getTarget_VelocityY(joystick_value);
    double vX = this.getTarget_VelocityX(joystick_value);

    Translation2d vector = new Translation2d(vX, vY);
    Rotation2d rot = new Rotation2d(Math.toRadians(m_CurrentHeading));

    vector = vector.rotateBy(rot);

    // return 0;
    return vector.getY();
  }

  public double getTarget_VelocityX_Adjusted(DoubleSupplier joystick_value) {

    // see if we should just use the joystick because we aren't on heading or too
    // far away, etc.
    double angleError = this.getTargetAngle().getDegrees() - m_CurrentHeading;

    DataLogManager.log(
        "Angle Error: " + angleError + " Found?: " + isTargetIDFound() + " Distance: " + m_TargetDistance.getDouble(3));

    if (Math.abs(angleError) > 5) {
      return joystick_value.getAsDouble();
    }
    if ((!isTargetIDFound())) {
      return joystick_value.getAsDouble();
    }
    if (Math.abs(m_TargetDistance.getDouble(5)) > 5.0) {
      return joystick_value.getAsDouble();
    }

    double vY = this.getTarget_VelocityY(joystick_value);
    double vX = this.getTarget_VelocityX(joystick_value);

    Translation2d vector = new Translation2d(vX, vY);
    Rotation2d rot = new Rotation2d(Math.toRadians(m_CurrentHeading));

    vector = vector.rotateBy(rot);

    return vector.getX();
  }

  private double getTarget_VelocityY(DoubleSupplier joystick_value) {

    double offset = -m_TargetOffset.getDouble(0.0);
    double answer = 0.0;

    double deadzone = 0.05; // TODO tune this
    double kP = 1.5; // TODO tune this
    double kS = 0.5; // TODO tune this

    if (offset < 0.0) {
      kS = kS * -1;
    }

    answer = (offset * kP) + kS;

    if (Math.abs(offset) < deadzone) {
      answer = 0;
    }
    DataLogManager.log("Target Left/Right: " + answer);

    return answer;
  }

  private double getTarget_VelocityX(DoubleSupplier joystick_value) {

    double answer = 0.0;

    double offset = getTargetDistanceError();

    double deadzone = 0.05; // TODO tune this
    double kP = 1.5; // TODO tune this
    double kS = 0.5; // TODO tune this

    if (offset < 0.0) {
      kS = kS * -1;
    }

    answer = (offset * kP) + kS;

    if (Math.abs(offset) < deadzone) {
      answer = 0;
    }

    DataLogManager.log("Target Front/Back: " + answer);

    return answer;
  }

  public Rotation2d getTargetAngle() {
    // double rot = m_TargetAngle.getDouble(-1.0);
    // if (head)
    return new Rotation2d(Math.toRadians(m_TargetAngle.getDouble(-1.0)));
  }

  public double getGamePiece_VelocityY() {
    double answer = 0;
    double offset = -m_GamePieceOffset.getDouble(0.0);

    double deadzone = 0.05; // TODO tune this
    double kP = 2.0; // TODO tune this
    double kS = 0.1; // TODO tune this

    if (offset < 0.0) {
      kS = kS * -1;
    }

    answer = (offset * kP) + kS;

    // if (answer <= 3) {
    // answer = 3;
    // }

    if (Math.abs(offset) < deadzone) {
      answer = 0;
    }

    DataLogManager.log("Game Piece Left/Right: " + answer);

    return answer;
  }

  public double getGamePiece_VelocityX() {
    double answer = 0;
    double offset = m_GamePieceDistance.getDouble(0.0);

    double deadzone = 0.05; // TODO tune this
    double kP = 1.0; // TODO tune this
    double kS = 0.1; // TODO tune this

    if (offset < 0.0) {
      kS = kS * -1;
    }

    answer = (offset * kP) + kS;

    if (answer <= 3) {
      answer = 3;
    }

    if (Math.abs(offset) < deadzone) {
      answer = 0;
    }

    DataLogManager.log("Game Piece Front/Back: " + answer);

    return answer;
  }

  public double getGamePiece_RotationalRate() {
    // if we don't see a game piece don't turn
    if (!this.isGamePieceFound()) {
      return 0;
    }

    double answer = 0;
    double offset = -Math.toRadians(m_LL_GamePiece.getEntry("tx").getDouble(0));

    double deadzone = 0.05; // TODO tune this
    double kP = 5; // TODO tune this
    double kS = 0.1; // TODO tune this

    if (offset < 0.0) {
      kS = kS * -1;
    }

    answer = (offset * kP) + kS;

    if (Math.abs(offset) < deadzone) {
      answer = 0;
    }

    DataLogManager.log("Game Piece Rotation: " + answer);

    return answer;
  }

  public void setTargetAngle(double degrees) {
    m_TargetAngle.setDouble(degrees);
  }

  public Command TargetTrackingMode() {
    Command c;

    c = new FunctionalCommand(
        () -> m_isTargetTracking = true,
        () -> RobotContainer.m_LEDs.targetTrackingMode(),
        interrupted -> m_isTargetTracking = false,
        () -> {
          return false;
        })
        .withName("TargetTrackingMode");

    return c;
  }

  public Command NoteTrackingMode() {
    Command c;

    c = new FunctionalCommand(
        () -> m_isNoteTracking = true,
        () -> RobotContainer.m_LEDs.noteTrackingMode(),
        interrupted -> m_isNoteTracking = false,
        () -> {
          return false;
        })
        .withName("NoteTrackingMode");

    return c;
  }

  public Command NoTrackingMode() {
    Command c;

    c = new FunctionalCommand(
        () -> {
          m_isNoteTracking = false;
          m_isTargetTracking = false;
        },
        () -> RobotContainer.m_LEDs.noTrackingMode(),
        interrupted -> m_isNoteTracking = false,
        () -> {
          return false;
        })
        .withName("NoTrackingMode");

    return c;
  }

}
