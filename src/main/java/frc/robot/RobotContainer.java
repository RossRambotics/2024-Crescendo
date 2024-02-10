// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import javax.sound.sampled.Line;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotMechanism;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tracking;

public class RobotContainer {
  private double MaxSpeed = 1; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.25 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick

  Trigger leftTrigger = new Trigger(
      () -> joystick.getRawAxis(XboxController.Axis.kLeftTrigger.value) >= 0.5);

  /* Setting up bindings for necessary control of the swerve drive platform */

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // private static final RobotMechanism m_mechRobot = new RobotMechanism();
  // public static final Indexer m_indexer = new Indexer();
  // public static final Intake m_intake = new Intake();
  // public static final Shooter m_shooter = new Shooter();
  public static final Indexer m_indexer = null;
  public static final Intake m_intake = null;
  public static final Shooter m_shooter = null;
  public static final Tracking m_tracking = new Tracking();
  private final SwerveRequest.FieldCentricFacingAngle targetDrive = new SwerveRequest.FieldCentricFacingAngle();
  private final SwerveRequest.RobotCentric gamePieceDrive = new SwerveRequest.RobotCentric();

  private static double slewLimit = 0.6;
  private static double rslewlimit = 0.3;
  private static double nudge = 0.7;
  private static double nudgeanglepower = .2;

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("S1 C1 S1");

  static public final Grabber m_grabber = new Grabber();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private SlewRateLimiter m_slewLeftY = new SlewRateLimiter(1.5);

  public double getInputLeftY() {
    double driverLeftY = modifyAxis(joystick.getLeftY());

    double slew = m_slewLeftY.calculate(driverLeftY * slewLimit)
        * 4.12;
    return slew;
  }

  private SlewRateLimiter m_slewLeftX = new SlewRateLimiter(1.5);

  public double getInputLeftX() {
    double driverLeftX = modifyAxis(joystick.getLeftX());

    double slew = m_slewLeftX.calculate(driverLeftX * slewLimit)
        * 4.12;
    return slew;
  }

  private SlewRateLimiter m_slewRightX = new SlewRateLimiter(1.5);

  public double getInputRightX() {
    double driverRightX = modifyAxis(joystick.getRightX());

    double slew = m_slewRightX.calculate(driverRightX * rslewlimit)
        * 4.12;
    return slew;
  }

  private void configureBindings() {
    Command cmd;

    leftTrigger.onTrue(Commands.runOnce(() -> slewLimit = 1.0));
    leftTrigger.onFalse(Commands.runOnce(() -> slewLimit = 0.6));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-getInputLeftY()) // Drive forward with
                                                                            // negative Y (forward)
            .withVelocityY(-getInputLeftX()) // Drive left with negative X (left)
            .withRotationalRate(-getInputRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    // left trigger invoke target tracking
    Rotation2d rot = new Rotation2d(Math.toRadians(90));

    joystick.leftBumper()
        .whileTrue(drivetrain.applyRequest(() -> targetDrive.withVelocityX(m_tracking.getTarget_VelocityX())
            .withVelocityY(m_tracking.getTarget_VelocityY())
            .withTargetDirection(rot)));

    // right trigger invoke game piece tracking
    joystick.rightBumper()
        .whileTrue(drivetrain.applyRequest(() -> gamePieceDrive.withVelocityX(m_tracking.getGamePiece_VelocityX())
            .withVelocityY(m_tracking.getGamePiece_VelocityY())
            .withRotationalRate(m_tracking.getGamePiece_RotationalRate())));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-getInputLeftY(), getInputLeftX()))));

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> drive
        .withVelocityX(nudge) // Drive forward with negative Y (forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(-getInputRightX() * MaxAngularRate * nudgeanglepower)));

    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> drive
        .withVelocityX(-nudge) // Drive forward with negative Y (forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(-getInputRightX() * MaxAngularRate * nudgeanglepower)));

    joystick.pov(90).whileTrue(drivetrain.applyRequest(() -> drive
        .withVelocityX(0) // Drive forward with negative Y (forward)
        .withVelocityY(-nudge) // Drive left with negative X (left)
        .withRotationalRate(-getInputRightX() * MaxAngularRate * nudgeanglepower)));

    joystick.pov(270).whileTrue(drivetrain.applyRequest(() -> drive
        .withVelocityX(0) // Drive forward with negative Y (forward)
        .withVelocityY(nudge) // Drive left with negative X (left)
        .withRotationalRate(-getInputRightX() * MaxAngularRate * nudgeanglepower)));

    // reset the field-centric heading on left bumper press
    joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
    // Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public RobotContainer() {
    configureBindings();

    /* Register named commands */
    NamedCommands.registerCommand("Open", new RunCommand(() -> m_grabber.openJaws()));
    NamedCommands.registerCommand("Close", new RunCommand(() -> m_grabber.closeJaws()));

  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
