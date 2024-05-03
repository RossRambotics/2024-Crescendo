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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Indexer.StoreOneNote;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.GridSelector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LClimb;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.RClimb;
import frc.robot.subsystems.RobotMechanism;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tracking;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // 6 meters per second desired top speed
        private double MaxAngularRate = 1.25 * Math.PI; // 3/4 of a rotation per second max angular velocity
        private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
        private final CommandXboxController operatorJoystick = new CommandXboxController(4);
        private double m_joystickAlliance = 1;

        Trigger leftTrigger = new Trigger(
                        () -> joystick.getRawAxis(XboxController.Axis.kLeftTrigger.value) >= 0.5);

        Trigger rightTrigger = new Trigger(
                        () -> joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) >= 0.5);

        Trigger operatorLeftUpTrigger = new Trigger(
                        () -> operatorJoystick.getRawAxis(XboxController.Axis.kLeftY.value) >= 0.5);

        Trigger operatorLeftDownTrigger = new Trigger(
                        () -> operatorJoystick.getRawAxis(XboxController.Axis.kLeftY.value) <= 0.5);

        Trigger operatorRightUpTrigger = new Trigger(
                        () -> operatorJoystick.getRawAxis(XboxController.Axis.kRightY.value) >= 0.5);

        Trigger operatorRightDownTrigger = new Trigger(
                        () -> operatorJoystick.getRawAxis(XboxController.Axis.kRightY.value) <= 0.5);

        Trigger aButton = new Trigger(joystick.a());
        Trigger bButton = new Trigger(joystick.b());

        SendableChooser<Command> m_autoChooser = new SendableChooser<>();

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
        public static final RClimb m_rClimb = new RClimb();
        public static final LClimb m_lClimb = new LClimb();
        public static final GridSelector m_gridSelector = new GridSelector();
        public static final LEDs m_LEDs = new LEDs();
        // public static final Indexer m_indexer = null;
        // public static final Intake m_intake = null;
        // public static final Shooter m_shooter = null;

        public static final Tracking m_tracking = new Tracking();
        private final SwerveRequest.FieldCentricFacingAngle targetDrive = new SwerveRequest.FieldCentricFacingAngle();
        private final SwerveRequest.RobotCentric gamePieceDrive = new SwerveRequest.RobotCentric();

        private static double slewLimit = 0.6;
        private static double rslewlimit = 0.3;
        private static double boostLimit = 0.3;
        private static double nudge = 0.7;
        private static double nudgeanglepower = .2;
        /* Path follower */
        private Command runAuto = null;

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final SlewRateLimiter m_slewLeftY = new SlewRateLimiter(5);

        public double getInputLeftY() {
                double driverLeftY = modifyAxis(joystick.getLeftY());

                double slew = m_slewLeftY.calculate(driverLeftY * slewLimit)
                                * 4.12;
                return slew;
        }

        private final SlewRateLimiter m_slewLeftX = new SlewRateLimiter(5);

        public double getInputLeftX() {
                double driverLeftX = modifyAxis(joystick.getLeftX());

                double slew = m_slewLeftX.calculate(driverLeftX * slewLimit)
                                * 4.12;
                return slew;
        }

        private SlewRateLimiter m_slewRightX = new SlewRateLimiter(6);

        public double getInputRightX() {
                double driverRightX = modifyAxis(joystick.getRightX());

                double slew = m_slewRightX.calculate(driverRightX * rslewlimit)
                                * 4.12;
                return slew;
        }

        private void configureBindings() {
                m_indexer.setDefaultCommand(new StoreOneNote());

                Command cmd;

                leftTrigger.onTrue(Commands.runOnce(() -> boostLimit = 1.4));
                leftTrigger.onFalse(Commands.runOnce(() -> boostLimit = 0.3));

                drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(
                                                () -> drive.withVelocityX(getInputLeftY() * MaxSpeed * boostLimit
                                                                * m_joystickAlliance) // Drive
                                                                                      // forward
                                                                // with
                                                                // negative Y (forward)

                                                                .withVelocityY(getInputLeftX() * MaxSpeed * boostLimit
                                                                                * m_joystickAlliance) // Drive
                                                                                                      // left
                                                                                                      // with
                                                                                                      // negative
                                                                                                      // X
                                                                                                      // (left)
                                                                .withRotationalRate(-getInputRightX() * MaxAngularRate) // Drive
                                                                                                                        // counterclockwise
                                                                                                                        // with
                                                                                                                        // negative
                                                                                                                        // X
                                                                                                                        // (left)
                                ).ignoringDisable(true)
                                                .alongWith(m_tracking.NoTrackingMode()));

                // left trigger invoke target tracking
                // Rotation2d rot = new Rotation2d(Math.toRadians(0.0));
                targetDrive.HeadingController.setP(5.0);
                targetDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
                targetDrive.HeadingController.setIntegratorRange(-0.25, 0.25);

                joystick.leftBumper()
                                .whileTrue(drivetrain
                                                .applyRequest(() -> targetDrive
                                                                .withVelocityX(
                                                                                m_tracking.getTarget_VelocityX_Adjusted(
                                                                                                () -> getInputLeftY()
                                                                                                                * m_joystickAlliance))
                                                                .withVelocityY(
                                                                                m_tracking.getTarget_VelocityY_Adjusted(
                                                                                                () -> getInputLeftX()
                                                                                                                * m_joystickAlliance))
                                                                .withTargetDirection(m_tracking.getTargetAngle()))
                                                .alongWith(m_tracking.TargetTrackingMode()));

                // right trigger invoke game piece tracking
                joystick.rightBumper()
                                .whileTrue(drivetrain
                                                .applyRequest(() -> gamePieceDrive
                                                                .withVelocityX(m_tracking.getGamePiece_VelocityX() / 2)
                                                                .withVelocityY(m_tracking.getGamePiece_VelocityY() / 2)
                                                                .withRotationalRate(m_tracking
                                                                                .getGamePiece_RotationalRate() / 2))
                                                .alongWith(m_tracking.NoteTrackingMode()));

                joystick.rightBumper().onTrue(new frc.robot.commands.Intake.Down()
                                .andThen(new frc.robot.commands.Intake.IntakeStart())
                                .andThen(new frc.robot.commands.Indexer.Stop())
                                .andThen(new frc.robot.commands.Shooter.Stop())
                                .andThen(new frc.robot.commands.Indexer.Intake())
                                .andThen(new WaitUntilCommand(() -> m_indexer.isNoteMiddle()))
                                .andThen(new frc.robot.commands.Intake.IntakeStop())
                                .andThen(new frc.robot.commands.Indexer.Stop())
                                .andThen(new frc.robot.commands.Intake.Up()));

                // deploy the intake
                joystick.a().onTrue(new frc.robot.commands.Intake.Down()
                                .andThen(new frc.robot.commands.Intake.IntakeStart())
                                .andThen(new frc.robot.commands.Indexer.Stop())
                                .andThen(new frc.robot.commands.Shooter.Stop())
                                .andThen(new frc.robot.commands.Indexer.Intake())
                                .andThen(new WaitUntilCommand(() -> m_indexer.isNoteMiddle()))
                                .andThen(new frc.robot.commands.Indexer.Stop())
                                .andThen(new frc.robot.commands.Intake.IntakeStop())
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
                Command shoot = new frc.robot.commands.Shooter.Start()
                                .andThen(new WaitUntilCommand(() -> m_shooter.isShooterReady()))
                                .andThen(new frc.robot.commands.Indexer.Shoot())
                                .andThen(new WaitCommand(1.0))
                                .andThen(new frc.robot.commands.Shooter.Stop())
                                .andThen(new frc.robot.commands.Indexer.Stop())
                                .withName("Shoot_a_Note");
                rightTrigger.onTrue(shoot);

                SmartDashboard.putData("Indexer.Shooter", shoot);
                /* */

                // joystick.x().onTrue(new frc.robot.commands.Shooter.Reverse()
                // .andThen(new frc.robot.commands.Indexer.Reverse()
                // .andThen(new WaitUntilCommand(() -> m_indexer.isNoteMiddle()))
                // .andThen(new frc.robot.commands.Indexer.Stop())
                // .andThen(new frc.robot.commands.Shooter.Stop())));

                // joystick.b().whileTrue(drivetrain
                // .applyRequest(() -> point.withModuleDirection(new
                // Rotation2d(-getInputLeftY(), getInputLeftX()))));

                joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> drive
                                .withVelocityX(nudge * m_joystickAlliance) // Drive forward with negative Y (forward)
                                .withVelocityY(0 * m_joystickAlliance) // Drive left with negative X (left)
                                .withRotationalRate(-getInputRightX() * MaxAngularRate * nudgeanglepower)));

                joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> drive
                                .withVelocityX(-nudge * m_joystickAlliance) // Drive forward with negative Y (forward)
                                .withVelocityY(0 * m_joystickAlliance) // Drive left with negative X (left)
                                .withRotationalRate(-getInputRightX() * MaxAngularRate * nudgeanglepower)));

                joystick.pov(90).whileTrue(drivetrain.applyRequest(() -> drive
                                .withVelocityX(0 * m_joystickAlliance) // Drive forward with negative Y (forward)
                                .withVelocityY(-nudge * m_joystickAlliance) // Drive left with negative X (left)
                                .withRotationalRate(-getInputRightX() * MaxAngularRate * nudgeanglepower)));

                joystick.pov(270).whileTrue(drivetrain.applyRequest(() -> drive
                                .withVelocityX(0 * m_joystickAlliance) // Drive forward with negative Y (forward)
                                .withVelocityY(nudge * m_joystickAlliance) // Drive left with negative X (left)
                                .withRotationalRate(-getInputRightX() * MaxAngularRate * nudgeanglepower)));

                // reset the field-centric heading on left bumper press
                // joystick.back().onTrue(drivetrain.runOnce(() ->
                // drivetrain.seedFieldRelative()));
                joystick.back().onTrue(drivetrain.runOnce(() -> this.resetFieldHeading()));

                // drivetrain.seedFieldRelative(new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)));
                this.resetFieldHeading();

                drivetrain.registerTelemetry(logger::telemeterize);

                m_gridSelector.initialize();

                // Operator controller Button Box broke
                // operatorJoystick.y().onTrue(new frc.robot.commands.Speaker.Middle());

                // operatorJoystick.x().onTrue(new frc.robot.commands.Speaker.Left());

                // operatorJoystick.b().onTrue(new frc.robot.commands.Speaker.Right());

                // operatorJoystick.a().onTrue(new frc.robot.commands.Amp.Shoot());

                // operatorLeftUpTrigger.whileTrue(new frc.robot.commands.LClimb.LClimbUp());

                // operatorLeftDownTrigger.whileTrue(new
                // frc.robot.commands.LClimb.LClimbDown());

                // //operatorLeftDownTrigger.whileFalse(new
                // frc.robot.commands.LClimb.LClimbStop());

                // operatorRightUpTrigger.whileTrue(new frc.robot.commands.RClimb.RClimbUp());

                // operatorRightDownTrigger.whileTrue(new
                // frc.robot.commands.RClimb.RClimbDown());

                // //operatorRightDownTrigger.whileFalse(new
                // frc.robot.commands.RClimb.RClimbStop());

                // operatorJoystick.leftBumper().onTrue(new
                // frc.robot.commands.Intake.IntakeReverse()
                // .andThen(new frc.robot.commands.Indexer.Intake()));

                // operatorJoystick.rightBumper().onTrue(new
                // frc.robot.commands.Shooter.Start());

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
                                m_joystickAlliance = 1;
                                return;
                        }
                }
                DataLogManager.log("%%%%%%%%%% resetFieldHeading: Blue.");
                drivetrain.seedFieldRelative(new Pose2d(1.3, 5.5,
                                Rotation2d.fromDegrees(0.0)));
                m_joystickAlliance = -1;
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
                NamedCommands.registerCommand("Indexer.Storage",
                                new frc.robot.commands.Indexer.StoreOneNote()
                                                .withTimeout(1.0));

                NamedCommands.registerCommand("Speaker.Middle",
                                new frc.robot.commands.Speaker.Middle()
                                                .andThen(new frc.robot.commands.Shooter.Start())
                                                .withName("Auto.Speaker.Middle"));

                NamedCommands.registerCommand("Amp.Shoot",
                                new frc.robot.commands.Amp.Shoot()
                                                .andThen(new frc.robot.commands.Shooter.Start())
                                                .withName("Auto.Amp.Shoot"));

                NamedCommands.registerCommand("Shooter.Start",
                                new frc.robot.commands.Shooter.Start()
                                                .withName("Auto.Shooter.Start"));

                NamedCommands.registerCommand("Shooter.Stop",
                                new frc.robot.commands.Shooter.Stop()
                                                .withName("Auto.Shooter.Stop"));

                NamedCommands.registerCommand("Indexer.Shoot.Fast",
                                new WaitUntilCommand(() -> m_shooter.isShooterReady())
                                                .andThen(new frc.robot.commands.Indexer.Shoot())
                                                .andThen(new WaitCommand(0.5))
                                                .andThen(new frc.robot.commands.Shooter.Stop())
                                                .andThen(new frc.robot.commands.Indexer.Stop())
                                                .withName("Auto.Indexer.Shoot.Fast"));

                NamedCommands.registerCommand("Indexer.Shoot",
                                new WaitUntilCommand(() -> m_shooter.isShooterReady())
                                                .andThen(new frc.robot.commands.Indexer.Shoot())
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new frc.robot.commands.Indexer.Stop())
                                                .andThen(new frc.robot.commands.Indexer.Intake())
                                                .withName("Auto.Indexer.Shoot"));

                NamedCommands.registerCommand("Intake.Down",
                                new frc.robot.commands.Intake.Down()
                                                .andThen(new frc.robot.commands.Intake.IntakeStart())
                                                .andThen(new frc.robot.commands.Indexer.Intake())
                                                .withName("Auto.Indexer.Down"));

                NamedCommands.registerCommand("Intake.Up",
                                new frc.robot.commands.Intake.IntakeStop()
                                                .andThen(new frc.robot.commands.Intake.Up())
                                                .withName("Auto.Indexer.Up"));

                NamedCommands.registerCommand("Indexer.Middle.Sensor",
                                new WaitUntilCommand(() -> m_indexer.isNoteMiddle()));

                NamedCommands.registerCommand("Pick.Up",
                                new frc.robot.commands.Intake.Down()
                                                .andThen(new frc.robot.commands.Intake.IntakeStart())
                                                .andThen(new frc.robot.commands.Indexer.Intake())
                                                .andThen(new WaitUntilCommand(() -> m_indexer.isNoteMiddle()))
                                                .andThen(new frc.robot.commands.Intake.IntakeStop())
                                                .andThen(new frc.robot.commands.Indexer.Stop())
                                                .andThen(new frc.robot.commands.Intake.Up()
                                                                .withName("Intake_a_Note")));

                Command autoDrive = drivetrain
                                .applyRequest(() -> gamePieceDrive
                                                .withVelocityX(m_tracking.getGamePiece_VelocityX() / 3)
                                                .withVelocityY(m_tracking.getGamePiece_VelocityY() / 3)
                                                .withRotationalRate(m_tracking.getGamePiece_RotationalRate()))
                                .alongWith(m_tracking.NoteTrackingMode());

                Command cmd = new frc.robot.commands.Intake.Down()
                                .andThen(new frc.robot.commands.Intake.IntakeStart())
                                .andThen(new frc.robot.commands.Indexer.Intake())
                                .andThen(drivetrain
                                                .applyRequest(() -> gamePieceDrive.withVelocityX(
                                                                m_tracking.getGamePiece_VelocityX()
                                                                                / 4)
                                                                .withVelocityY(m_tracking
                                                                                .getGamePiece_VelocityY()
                                                                                / 4)
                                                                .withRotationalRate(m_tracking
                                                                                .getGamePiece_RotationalRate()
                                                                                / 4))
                                                .alongWith(m_tracking.NoteTrackingMode()))
                                .until(() -> m_indexer.isNoteBottom())
                                // .andThen(new frc.robot.commands.Intake.IntakeStop())
                                // .andThen(drivetrain.applyRequest(() -> drive
                                // .withVelocityX(0).withVelocityY(0)
                                // .withRotationalRate(0)))
                                // .until(() -> m_indexer.isNoteBottom())
                                .withName("Auto Intake Note");

                NamedCommands.registerCommand("AUTO.NOTE.TRACKING", cmd);

                NamedCommands.registerCommand("Auto.Pick.Up",
                                new frc.robot.commands.Intake.Down()
                                                .andThen(new frc.robot.commands.Intake.IntakeStart()
                                                                .andThen(new frc.robot.commands.Indexer.Intake())
                                                                .andThen(drivetrain
                                                                                .applyRequest(() -> gamePieceDrive
                                                                                                .withVelocityX(m_tracking
                                                                                                                .getGamePiece_VelocityX()
                                                                                                                / 2)
                                                                                                .withVelocityY(m_tracking
                                                                                                                .getGamePiece_VelocityY()
                                                                                                                / 2)
                                                                                                .withRotationalRate(
                                                                                                                m_tracking
                                                                                                                                .getGamePiece_RotationalRate()
                                                                                                                                / 2))
                                                                                .alongWith(m_tracking
                                                                                                .NoteTrackingMode()))
                                                                .withTimeout(3)
                                                                .withName("Auto Intake Note")));

                SmartDashboard.putData("Auto.Intake.Note", cmd);

                try {
                        m_autoChooser.setDefaultOption("Dont Move", new WaitCommand(1.0));
                        m_autoChooser.addOption("2M Straight", drivetrain.getAutoPath("2M Straight"));
                        m_autoChooser.addOption("S2 C1 C2 C3", drivetrain.getAutoPath("S2 C1 C2 C3"));
                        // m_autoChooser.addOption("S1 C1 C2 F1", drivetrain.getAutoPath("S1 C1 C2
                        // F1"));
                        // m_autoChooser.addOption("S1 C1 C2 F2", drivetrain.getAutoPath("S1 C1 C2
                        // F2"));
                        // m_autoChooser.addOption("S1 C1 C2", drivetrain.getAutoPath("S1 C1 C2"));
                        m_autoChooser.addOption("S1 C1 F1 F2", drivetrain.getAutoPath("S1 C1 F1 F2"));
                        // m_autoChooser.addOption("S1 C1 F1", drivetrain.getAutoPath("S1 C1 F1"));
                        // m_autoChooser.addOption("S1 C1 F2", drivetrain.getAutoPath("S1 C1 F2"));
                        // m_autoChooser.addOption("S1 C1", drivetrain.getAutoPath("S1 C1"));
                        // m_autoChooser.addOption("S1 C2", drivetrain.getAutoPath("S1 C2"));
                        // m_autoChooser.addOption("S1 C3", drivetrain.getAutoPath("S1 C3"));
                        // m_autoChooser.addOption("S2 C1", drivetrain.getAutoPath("S2 C1"));
                        m_autoChooser.addOption("Note Trackng Test", drivetrain.getAutoPath("Note Trackng Test"));
                        // m_autoChooser.addOption("S2 C2 C1 F3", drivetrain.getAutoPath("S2 C2 C1
                        // F3"));
                        // m_autoChooser.addOption("S2 C2 C1", drivetrain.getAutoPath("S2 C2 C1"));
                        m_autoChooser.addOption("S2 C2 C3 F3", drivetrain.getAutoPath("S2 C2 C3 F3"));
                        // m_autoChooser.addOption("S2 C2 C3", drivetrain.getAutoPath("S2 C2 C3"));
                        // m_autoChooser.addOption("S2 C2 F2", drivetrain.getAutoPath("S2 C2 F2"));
                        // m_autoChooser.addOption("S2 C2 F3", drivetrain.getAutoPath("S2 C2 F3"));
                        // m_autoChooser.addOption("S2 C2", drivetrain.getAutoPath("S2 C2"));
                        // m_autoChooser.addOption("S2 C3", drivetrain.getAutoPath("S2 C3"));
                        // m_autoChooser.addOption("S2 F2 F1", drivetrain.getAutoPath("S2 F2 F1"));
                        m_autoChooser.addOption("Amp F1 F2 F3 F4 F5", drivetrain.getAutoPath("Amp F1 F2 F3 F4 F5"));
                        // m_autoChooser.addOption("S3 C3 C2 C1", drivetrain.getAutoPath("S3 C3 C2
                        // C1"));
                        m_autoChooser.addOption("T S2 F2", drivetrain.getAutoPath("T S3 F4 F5"));
                        // m_autoChooser.addOption("S3 C3 F4 F5", drivetrain.getAutoPath("S3 C3 F4
                        // F5"));
                        m_autoChooser.addOption("Shoot NO MOVE", drivetrain.getAutoPath("Shoot NO MOVE")
                                        .withTimeout(4));
                        m_autoChooser.addOption("S2 C2 F1 C1 C3 F3", drivetrain.getAutoPath("S2 C2 F1 C1 C3 F3"));
                        m_autoChooser.addOption("S2 C2 F3 C1 C3 F3", drivetrain.getAutoPath("S2 C2 F3 C1 C3 F3"));
                        m_autoChooser.addOption("S2 C2 C3 C1 F2 F3", drivetrain.getAutoPath("S2 C2 C3 C1 F2 F3"));
                        m_autoChooser.addOption("S2 C2 C3 C1 F3 F3", drivetrain.getAutoPath("S2 C2 C3 C1 F3 F3"));
                        // m_autoChooser.addOption("S2 C1 C2 C3 F2", drivetrain.getAutoPath("S2 C1 C2 C3
                        // F2"));
                        m_autoChooser.addOption("S3 F4 F5", drivetrain.getAutoPath("S3 F4 F5"));

                } catch (Exception e) {
                        DataLogManager.log("Missing auto file.");
                        DataLogManager.log(e.toString());
                }
                SmartDashboard.putData(m_autoChooser);

                configureBindings();
                // LiveWindow.enableTelemetry(m_indexer);
                // LiveWindow.enableTelemetry(m_intake);
                // LiveWindow.enableTelemetry(m_shooter);
                // LiveWindow.enableTelemetry(m_tracking);

                SmartDashboard.putData("Robot.ResetPose", new InstantCommand(() -> this.resetFieldHeading()));

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
                return m_autoChooser.getSelected();
        }

}
