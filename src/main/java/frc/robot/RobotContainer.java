// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Indexer.Storage;
import frc.robot.commands.Intake.Down;
import frc.robot.generated.TunerConstants;
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

  Trigger rightTrigger = new Trigger(
      () -> joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) >= 0.5);
  Trigger aButton = new Trigger(joystick.a());
  Trigger bButton = new Trigger(joystick.b());

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

  private static final RobotMechanism m_mechRobot = new RobotMechanism();
  public static final Indexer m_indexer = new Indexer();
  public static final Intake m_intake = new Intake();
  public static final Shooter m_shooter = new Shooter();
  // public static final Indexer m_indexer = null;
  // public static final Intake m_intake = null;
  // public static final Shooter m_shooter = null;

  public static final Tracking m_tracking = new Tracking();
  private final SwerveRequest.FieldCentricFacingAngle targetDrive = new SwerveRequest.FieldCentricFacingAngle();
  private final SwerveRequest.RobotCentric gamePieceDrive = new SwerveRequest.RobotCentric();

  private static double slewLimit = 0.6;
  private static double rslewlimit = 0.3;
  private static double nudge = 0.7;
  private static double nudgeanglepower = .2;

  /* Path follower */
  private Command runAuto = null;

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
    m_indexer.setDefaultCommand(new Storage());

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
    // Rotation2d rot = new Rotation2d(Math.toRadians(0.0));
    targetDrive.HeadingController.setP(3.0);
    targetDrive.HeadingController.enableContinuousInput(0, 360);
    targetDrive.HeadingController.setIntegratorRange(-0.25, 0.25);

    joystick.leftBumper()
        .whileTrue(drivetrain
            .applyRequest(() -> targetDrive.withVelocityX(m_tracking.getTarget_VelocityX(() -> -getInputLeftY()))
                .withVelocityY(m_tracking.getTarget_VelocityY(() -> -getInputLeftX()))
                .withTargetDirection(m_tracking.getTargetAngle()))
            .alongWith(m_tracking.TargetTrackingMode()));

    // right trigger invoke game piece tracking
    joystick.rightBumper()
        .whileTrue(drivetrain.applyRequest(() -> gamePieceDrive.withVelocityX(m_tracking.getGamePiece_VelocityX())
            .withVelocityY(m_tracking.getGamePiece_VelocityY())
            .withRotationalRate(m_tracking.getGamePiece_RotationalRate()))
            .alongWith(m_tracking.NoteTrackingMode()));

    // deploy the intake
    joystick.a().onTrue(new frc.robot.commands.Intake.Down()
        .andThen(new frc.robot.commands.Intake.IntakeStart())
        .andThen(new frc.robot.commands.Indexer.Intake())
        .andThen(new WaitUntilCommand(() -> m_indexer.isNoteMiddle()))
        .andThen(new frc.robot.commands.Intake.IntakeStop())
        .andThen(new frc.robot.commands.Indexer.Stop())
        .andThen(new frc.robot.commands.Intake.Up()
            .withName("Intake_a_Note")
        /* */));

    // configure for speaker
    joystick.y().onTrue(new frc.robot.commands.Speaker.Middle()
        .withName("Speaker.Middle")
    /* */);

    joystick.x().onTrue(new frc.robot.commands.Amp.Shoot()
        .withName("Amp.Shoot")
    /* */);

    joystick.b().onTrue(new frc.robot.commands.Intake.Up()
        .andThen(new frc.robot.commands.Intake.IntakeStop())
        .andThen(new frc.robot.commands.Indexer.Stop())
        .andThen(new frc.robot.commands.Shooter.Stop()));

    // shoot
    rightTrigger.onTrue(new frc.robot.commands.Shooter.Start()
        .andThen(new WaitUntilCommand(() -> m_shooter.isShooterReady()))
        .andThen(new frc.robot.commands.Indexer.Shoot())
        .andThen(new WaitCommand(1.0))
        .andThen(new frc.robot.commands.Shooter.Stop())
        .andThen(new frc.robot.commands.Indexer.Stop())
        .withName("Shoot_a_Note")
    /* */);

    // joystick.x().onTrue(new frc.robot.commands.Shooter.Reverse()
    // .andThen(new frc.robot.commands.Indexer.Reverse()
    // .andThen(new WaitUntilCommand(() -> m_indexer.isNoteMiddle()))
    // .andThen(new frc.robot.commands.Indexer.Stop())
    // .andThen(new frc.robot.commands.Shooter.Stop())));

    // joystick.b().whileTrue(drivetrain
    // .applyRequest(() -> point.withModuleDirection(new
    // Rotation2d(-getInputLeftY(), getInputLeftX()))));

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
    // joystick.back().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldRelative()));
    joystick.back().onTrue(drivetrain.runOnce(() -> this.resetFieldHeading()));

    // drivetrain.seedFieldRelative(new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)));
    this.resetFieldHeading();

    drivetrain.registerTelemetry(logger::telemeterize);

  }

  /*
   * reset the robot heading based on the alliance color
   * the robot front should be facing straight away from the operator when using
   */
  private void resetFieldHeading() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        DataLogManager.log("%%%%%%%%%% resetFieldHeading: Red.");
        drivetrain.seedFieldRelative(new Pose2d(15.25, 5.5,
            Rotation2d.fromDegrees(180)));
        return;
      }
    }
    DataLogManager.log("%%%%%%%%%% resetFieldHeading: Blue.");
    drivetrain.seedFieldRelative(new Pose2d(1.3, 5.5,
        Rotation2d.fromDegrees(0.0)));
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

    // Named Commands must be created BEFORE AUTOs and PATHs!!!!!!!!!!!!!!!!!!!!!
    NamedCommands.registerCommand("Speaker.Middle",
        new frc.robot.commands.Speaker.Middle()
            .andThen(new frc.robot.commands.Shooter.Start())
            .withName("Auto.Speaker.Middle"));

    NamedCommands.registerCommand("Shooter.Start",
        new frc.robot.commands.Shooter.Start()
            .withName("Auto.Shooter.Start"));

    NamedCommands.registerCommand("Indexer.Shoot.Fast",
        new WaitUntilCommand(() -> m_shooter.isShooterReady())
            .andThen(new frc.robot.commands.Indexer.Shoot())
            .andThen(new WaitCommand(0.5))
            .andThen(new frc.robot.commands.Shooter.Stop())
            .andThen(new frc.robot.commands.Indexer.Stop())
            .withName("Auto.Indexer.Shoot.Fast"));

    NamedCommands.registerCommand("Indexer.Shoot",
        new frc.robot.commands.Shooter.Start()
            .andThen(new WaitUntilCommand(() -> m_shooter.isShooterReady()))
            .andThen(new frc.robot.commands.Indexer.Shoot())
            .andThen(new WaitCommand(0.5))
            .andThen(new frc.robot.commands.Shooter.Stop())
            .andThen(new frc.robot.commands.Indexer.Stop())
            .withName("Auto.Indexer.Shoot"));

    NamedCommands.registerCommand("Intake.Down",
        new frc.robot.commands.Intake.Down()
            .andThen(new frc.robot.commands.Intake.IntakeStart())
            .withName("Auto.Indexer.Down"));

    NamedCommands.registerCommand("Intake.Up",
        new frc.robot.commands.Intake.IntakeStop()
            .andThen(new frc.robot.commands.Intake.Up())
            .withName("Auto.Indexer.Up"));

    NamedCommands.registerCommand("Pick.Up",
        new frc.robot.commands.Intake.Down()
            .andThen(new frc.robot.commands.Intake.IntakeStart())
            .andThen(new frc.robot.commands.Indexer.Intake())
            .andThen(new WaitUntilCommand(() -> m_indexer.isNoteMiddle()))
            .andThen(new frc.robot.commands.Intake.IntakeStop())
            .andThen(new frc.robot.commands.Indexer.Stop())
            .andThen(new frc.robot.commands.Intake.Up()
                .withName("Intake_a_Note")));

    runAuto = drivetrain.getAutoPath("New Auto");

    configureBindings();
    LiveWindow.enableTelemetry(m_indexer);
    LiveWindow.enableTelemetry(m_intake);
    LiveWindow.enableTelemetry(m_shooter);
    LiveWindow.enableTelemetry(m_tracking);

    SmartDashboard.putData("Robot.ResetPose", new InstantCommand(() -> this.resetFieldHeading()));

    m_intake.startCompresser();

    /* Register named commands */
    // NamedCommands.registerCommand("Open", new RunCommand(() ->
    // m_grabber.openJaws()));
    // NamedCommands.registerCommand("Close", new RunCommand(() ->
    // m_grabber.closeJaws()));

    // .andThen(new RunCommand(() -> m_intake.intake()))
    // .andThen(new frc.robot.commands.Indexer.Intake())
    // .andThen(new WaitUntilCommand(() -> m_indexer.isNoteMiddle()))
    // .andThen(new frc.robot.commands.Intake.IntakeStop())
    // .andThen(new frc.robot.commands.Indexer.Stop())
    // .andThen(new frc.robot.commands.Intake.Up())

  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
