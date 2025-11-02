// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.Side;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.chute.ChuteIO;
import frc.robot.subsystems.chute.ChuteIOReal;
import frc.robot.subsystems.chute.ChuteIOSim;
import frc.robot.subsystems.chuterShooter.ChuterShooter;
import frc.robot.subsystems.chuterShooter.ChuterShooterIO;
import frc.robot.subsystems.chuterShooter.ChuterShooterIOReal;
import frc.robot.subsystems.chuterShooter.ChuterShooterIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Superstructure superstructure;
  private final ChuterShooter chuterShooter;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final GenericHID operator1Controller = new GenericHID(1);
  private final GenericHID operator2Controller = new GenericHID(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(DriveConstants.swerveModuleConfigsComp[0]), // FL
                new ModuleIOSpark(DriveConstants.swerveModuleConfigsComp[1]), // FR
                new ModuleIOSpark(DriveConstants.swerveModuleConfigsComp[2]), // BL
                new ModuleIOSpark(DriveConstants.swerveModuleConfigsComp[3])); // BR
        superstructure =
            new Superstructure(new Elevator(new ElevatorIOReal()), new Chute(new ChuteIOReal()));
        chuterShooter = new ChuterShooter(new ChuterShooterIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        superstructure =
            new Superstructure(new Elevator(new ElevatorIOSim(0)), new Chute(new ChuteIOSim()));
        chuterShooter = new ChuterShooter(new ChuterShooterIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        superstructure =
            new Superstructure(new Elevator(new ElevatorIO() {}), new Chute(new ChuteIO() {}));
        chuterShooter = new ChuterShooter(new ChuterShooterIO() {});
        break;
    }

    // Named Commands
    NamedCommands.registerCommand(
        "Home",
        Commands.runOnce(
            () -> {
              superstructure.elevator.home();
              superstructure.chute.home();
              Logger.recordOutput("Auto/RanHome", true);
            }));

    NamedCommands.registerCommand(
        "SetCenter",
        Commands.runOnce(
            () -> {
              Pose2d pose = new Pose2d(new Translation2d(7.11, 4), Rotation2d.fromDegrees(120));
              if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                pose = FlippingUtil.flipFieldPose(pose);
              }
              drive.setPose(pose);
              Logger.recordOutput("Auto/SetCenter", pose);
            }));

    NamedCommands.registerCommand(
        "SetStation1",
        Commands.runOnce(
            () -> {
              Pose2d pose = new Pose2d(new Translation2d(7.120, 2), Rotation2d.fromDegrees(90));
              if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                pose = FlippingUtil.flipFieldPose(pose);
              }
              drive.setPose(pose);
              Logger.recordOutput("Auto/SetStation3", pose);
            }));

    NamedCommands.registerCommand(
        "SetStation3",
        Commands.runOnce(
            () -> {
              Pose2d pose = new Pose2d(new Translation2d(7.120, 2), Rotation2d.fromDegrees(60));
              if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                pose = FlippingUtil.flipFieldPose(pose);
              }
              drive.setPose(pose);
              Logger.recordOutput("Auto/SetStation3", pose);
            }));

    NamedCommands.registerCommand(
        "ScoreL1",
        Commands.sequence(
            Commands.runOnce(
                () -> superstructure.gotoSetpoint(CoralLevel.L1, Side.RIGHT), superstructure),
            Commands.race(Commands.waitUntil(() -> superstructure.atGoal()), Commands.waitSeconds(2)),
            Commands.runOnce(() -> chuterShooter.startShooting(), chuterShooter),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> chuterShooter.stopShooting(), chuterShooter)));

    NamedCommands.registerCommand(
        "ScoreL2",
        Commands.sequence(
            Commands.runOnce(
                () -> superstructure.gotoSetpoint(CoralLevel.L2, Side.RIGHT), superstructure),
            Commands.waitUntil(() -> superstructure.atGoal()),
            Commands.runOnce(() -> chuterShooter.startShooting(), chuterShooter),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> chuterShooter.stopShooting(), chuterShooter)));

    NamedCommands.registerCommand(
        "ScoreL3",
        Commands.sequence(
            Commands.runOnce(
                () -> superstructure.gotoSetpoint(CoralLevel.L3, Side.RIGHT), superstructure),
            Commands.waitUntil(() -> superstructure.atGoal()),
            Commands.runOnce(() -> chuterShooter.startShooting(), chuterShooter),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> chuterShooter.stopShooting(), chuterShooter)));

    NamedCommands.registerCommand(
        "IntakeRight",
        Commands.sequence(
            Commands.runOnce(
                () -> superstructure.gotoSetpoint(CoralLevel.Intake, Side.RIGHT), superstructure),
            Commands.waitUntil(() -> superstructure.atGoal())));

    NamedCommands.registerCommand(
        "IntakeLeft",
        Commands.sequence(
            Commands.runOnce(
                () -> superstructure.gotoSetpoint(CoralLevel.Intake, Side.LEFT), superstructure),
            Commands.waitUntil(() -> superstructure.atGoal())));

    NamedCommands.registerCommand(
        "Fold", Commands.runOnce(() -> superstructure.foldForClimp(), superstructure));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("First Auto", DriveCommands.firstAuto(drive));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Vision subsystem
    vision =
        new Vision(
            (tid) -> {
              this.controller.setRumble(RumbleType.kBothRumble, 0.5);
              this.controller.setRumble(RumbleType.kBothRumble, 0);
            });

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * 0.7,
            () -> -controller.getLeftX() * 0.7,
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY() * 0.7,
                () -> -controller.getLeftX() * 0.7,
                () -> Rotation2d.fromDegrees(-90))); // new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // we are assuming the driver is sensible, and will go to the tag they last saw close to when
    // they see it
    controller
        .y()
        .whileTrue(
            DriveCommands.joystickDriveToPose(
                drive,
                () -> {
                  var targetTag = this.vision.getLastSeenTagID();
                  if (targetTag == -1) {
                    return null;
                  }
                  switch (DriveCommands.targetSide) {
                    case 1: // center
                      return DriveCommands.getLandingPose(targetTag);
                    case 2: // right
                      return DriveCommands.getRightLandingPose(targetTag);
                    case 0: // left
                    default:
                      return DriveCommands.getLeftLandingPose(targetTag);
                  }
                }));

    controller
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY() * 0.5,
                () -> -controller.getLeftX() * 0.5,
                () -> -controller.getRightX() * 0.7));

    controller
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

    new JoystickButton(operator2Controller, 1)
        .whileTrue(
            Commands.run(
                () -> {
                  superstructure.setElevatorGoalHeightMillimeters(
                      superstructure.elevator.getHeightMillimeters() + 60);
                },
                superstructure));

    new JoystickButton(operator2Controller, 2)
        .whileTrue(
            Commands.run(
                () -> {
                  superstructure.setElevatorGoalHeightMillimeters(
                      superstructure.elevator.getHeightMillimeters() - 60);
                },
                superstructure));

    new JoystickButton(operator2Controller, 3)
        .whileTrue(
            Commands.run(
                () -> {
                  if (superstructure.getElevatorHeightMillimeters()
                      > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM) {
                    superstructure.setChutePivotGoalRads(
                        superstructure.chute.getPivotAngleRads() + 0.4);
                  }
                },
                superstructure))
        .onFalse(
            Commands.runOnce(
                () -> {
                  superstructure.chute.setPivotGoalRads(superstructure.chute.getPivotAngleRads());
                },
                superstructure));

    new JoystickButton(operator2Controller, 8)
        .whileTrue(
            Commands.run(
                () -> {
                  if (superstructure.getElevatorHeightMillimeters()
                      > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM) {
                    superstructure.setChutePivotGoalRads(
                        superstructure.chute.getPivotAngleRads() - 0.4);
                  }
                },
                superstructure))
        .onFalse(
            Commands.runOnce(
                () -> {
                  superstructure.chute.setPivotGoalRads(superstructure.chute.getPivotAngleRads());
                },
                superstructure));

    new JoystickButton(operator2Controller, 5)
        .whileTrue(
            Commands.run(
                () -> {
                  chuterShooter.startShooting();
                },
                chuterShooter))
        .onFalse(
            Commands.runOnce(
                () -> {
                  chuterShooter.setShooterMotor(0);
                },
                chuterShooter));

    new JoystickButton(operator2Controller, 7)
        .whileTrue(
            Commands.run(
                () -> {
                  chuterShooter.setShooterMotor(-0.1); // intake
                },
                chuterShooter))
        .onFalse(
            Commands.runOnce(
                () -> {
                  chuterShooter.stopShooting();
                },
                chuterShooter));

    new JoystickButton(operator1Controller, 9)
        .onTrue(
            Commands.runOnce(
                () -> {
                  superstructure.home();
                },
                superstructure));

    new JoystickButton(operator1Controller, 14)
        .onTrue(
            Commands.runOnce(
                () -> {
                  superstructure.foldForClimp();
                },
                superstructure));

    new JoystickButton(operator1Controller, 4)
        .onTrue(
            Commands.runOnce(
                () -> {
                  superstructure.gotoSetpoint(CoralLevel.L3, Side.RIGHT);
                },
                superstructure));
    new JoystickButton(operator1Controller, 6)
        .onTrue(
            Commands.runOnce(
                () -> {
                  superstructure.gotoSetpoint(CoralLevel.L3, Side.LEFT);
                },
                superstructure));
    new JoystickButton(operator1Controller, 3)
        .onTrue(
            Commands.runOnce(
                () -> {
                  superstructure.gotoSetpoint(CoralLevel.L2, Side.RIGHT);
                },
                superstructure));
    new JoystickButton(operator1Controller, 8)
        .onTrue(
            Commands.runOnce(
                () -> {
                  superstructure.gotoSetpoint(CoralLevel.L2, Side.LEFT);
                },
                superstructure));
    new JoystickButton(operator1Controller, 5)
        .onTrue(
            Commands.runOnce(
                () -> {
                  superstructure.gotoSetpoint(CoralLevel.L1, Side.LEFT);
                },
                superstructure));
    new JoystickButton(operator1Controller, 7)
        .onTrue(
            Commands.runOnce(
                () -> {
                  superstructure.gotoSetpoint(CoralLevel.L1, Side.RIGHT);
                },
                superstructure));

    new JoystickButton(operator1Controller, 12)
        .onTrue(
            Commands.sequence(
                new ProxyCommand(
                    Commands.runOnce(
                        () -> superstructure.gotoSetpoint(CoralLevel.Intake, Side.LEFT),
                        superstructure))
                // new ProxyCommand(chuterShooter.loadCoralChute())
                ));

    new JoystickButton(operator1Controller, 11)
        .onTrue(
            Commands.sequence(
                new ProxyCommand(
                    Commands.runOnce(
                        () -> superstructure.gotoSetpoint(CoralLevel.Intake, Side.RIGHT),
                        superstructure))
                // new ProxyCommand(chuterShooter.loadCoralChute())
                ));
    new POVButton(operator2Controller, 90)
        .onTrue(
            Commands.runOnce(
                () -> {
                  DriveCommands.targetSide = 2; // right
                }));
    new POVButton(operator2Controller, 180)
        .onTrue(
            Commands.runOnce(
                () -> {
                  DriveCommands.targetSide = 1; // center
                }));
    new POVButton(operator2Controller, 270)
        .onTrue(
            Commands.runOnce(
                () -> {
                  DriveCommands.targetSide = 0; // left
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
