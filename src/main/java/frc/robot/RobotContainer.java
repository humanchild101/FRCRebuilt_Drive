// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.factories.RobotFactory;
import frc.robot.commands.factories.ShooterFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autos.AutoRoutineFactory;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants.ClimbPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.AnglerPosition;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterVoltage;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants.TurretPosition;
import frc.robot.subsystems.turretVision.TurretVisionConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.LoggedCommandScheduler;
import frc.robot.util.ScorePositionsUtil;

public class RobotContainer {
  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Vision vision;
  private final Climb climb;
  private final Turret turret;

  private final CommandXboxController controller = new CommandXboxController(0);

  private final AutoChooser autoChooser;
  private final LoggedDashboardChooser<Command> sysIdChooser;
  private final LoggedNetworkBoolean scoreConfirmOverride;
  private final ScorePositionsUtil scorePositionsUtil;

  private final Field2d field;

  // State for right-trigger double-press (2 presses in 2 seconds)
  private int rightTriggerPressCount = 0;
  private double rightTriggerFirstPressTime = 0;
  private boolean lastRightTriggerPressed = false;

  public RobotContainer() {
	switch (Constants.currentMode) {
			case REAL:
				// Real robot, instantiate hardware IO implementations
				drive = new Drive(
						new GyroIOReal(),
						new ModuleIOReal(TunerConstants.FrontLeft),
						new ModuleIOReal(TunerConstants.FrontRight),
						new ModuleIOReal(TunerConstants.BackLeft),
						new ModuleIOReal(TunerConstants.BackRight));
				intake = new Intake(new IntakeIOReal());
				indexer = new Indexer(new IndexerIOReal());
				shooter = new Shooter(new ShooterIOReal());
				vision = new Vision(
						drive::addVisionMeasurement,
						drive::addFilteredVisionMeasurement,
						new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
						new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1),
						new VisionIOPhotonVision(TurretVisionConstants.camera3Name, TurretVisionConstants.robotToCamera3));
				/* climb = new Climb(new ClimbIOReal()); */
				scorePositionsUtil = new ScorePositionsUtil(drive::getFieldPose);
				break;
			default:
				// Replayed robot, disable IO implementations
				drive = new Drive(
						new GyroIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						});
				intake = new Intake(new IntakeIO() {
				});
				indexer = new Indexer(new IndexerIO() {
				});
				shooter = new Shooter(new ShooterIO() {
				});
				vision = new Vision(
						drive::addVisionMeasurement,
						drive::addFilteredVisionMeasurement,
						new VisionIO() {
						},
						new VisionIO() {
						},
						new VisionIO() {
						});
				/*
				 * climb = new Climb(new ClimbIO() {
				 * });
				 */
				scorePositionsUtil = new ScorePositionsUtil(drive::getFieldPose);
				break;
		}

		// Setup Command Usage Logging
		LoggedCommandScheduler.init(CommandScheduler.getInstance());

		// Set up dashboard choosers
		autoChooser = new AutoChooser();
		sysIdChooser = new LoggedDashboardChooser<>("SysId Choices");

		configureAutos();
		configureSysId();

		
		scoreConfirmOverride = new LoggedNetworkBoolean("Confirm Scoring", false);
		
		// Configure Dashboard Display
		field = new Field2d();
		field.setRobotPose(drive.getFieldPose());
		SmartDashboard.putData("Field", field);

		// Configure the button bindings
		configureBindings();
  }
  private void configureAutos() {
		SmartDashboard.putData("Auto Chooser", autoChooser);
		AutoRoutineFactory autos = new AutoRoutineFactory(drive, intake,  shooter, climb, indexer, turret);
		autoChooser.addRoutine("Left Score", autos :: LeftScore);
		autoChooser.addRoutine("Right Score", autos :: RightScore);
		autoChooser.addRoutine("Left", autos :: Left);
		autoChooser.addRoutine("Right", autos :: Right);

		RobotModeTriggers.autonomous().whileTrue(
				Commands.runOnce(this::resetState).andThen(autoChooser.selectedCommandScheduler()));
	}

  public void update(){
    field.setRobotPose(drive.getFieldPose());
		LoggedCommandScheduler.periodic();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
				DriveCommands.joystickDrive(
						drive,
						() -> -controller.getLeftY(),
						() -> -controller.getLeftX(),
						() -> -controller.getRightX()));
    controller
				.b() //Starting button
				.and(() -> isFree(shooter, indexer, intake, climb))
				.and(() -> intake.getIntakePosition() == IntakePosition.START && 
				 turret.getTargetDir() == TurretPosition.START 
				&& shooter.getTargetPos() == AnglerPosition.START
				&& climb.getTargetPosition() == ClimbPosition.START
				).onTrue(RobotFactory.start(intake, shooter, null, turret));
    controller
				.y()
				.and(() -> isFree(shooter, turret, indexer))
				.onTrue(
					RobotFactory.score(shooter, indexer, turret, () -> ShooterVoltage.MID));
    controller
				.x()
				.and(() -> isFree(shooter, turret, indexer))
				.onTrue(RobotFactory.pass(() -> false, indexer, intake, shooter)); //Wanna do another multipress here
	controller
				.x()
				.and(() -> isFree(shooter, turret, indexer))
				.multiPress(2,2)
				.onTrue(RobotFactory.pass(() -> true, indexer, intake, shooter)); 
    controller
				.a();
    controller
				.start()
				.onTrue(
						Commands.runOnce(
								() -> drive.setPose(
										new Pose2d(drive.getFieldPose().getTranslation(), new Rotation2d())),
								drive)
								.ignoringDisable(true));
    //back buttons of the controller. Trigger is the furthest back
    controller
				.leftBumper()
				.onTrue(cancel());
    controller
				.leftTrigger()
				.toggleOnTrue(RobotFactory.intakeFuel(intake));
    controller
				.rightBumper();
				
	controller
				.rightTrigger()
				.and(() -> isFree(shooter))
				.onTrue(ShooterFactory.angle(shooter, () -> AnglerPosition.DOWN));
	controller
				.rightTrigger()
				.and(() -> isFree(shooter))
				.multiPress(2, 2)
				.onTrue(ShooterFactory.angle(shooter, () -> AnglerPosition.UP));

				/* 
	// Double-press (2 clicks in 2 seconds) for angle UP — multiPress() is not in WPILib 2025
		new Trigger(this::rightTriggerDoublePress)
				.and(() -> isFree(shooter))
				.multiPress(2,2) //2 clicks in 2 secs
				.onTrue(ShooterFactory.angle(shooter, () -> AnglerPosition.UP));
				*/

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /** Returns true once when right trigger is pressed twice within 2 seconds. 
  private boolean rightTriggerDoublePress() {
    boolean pressed = controller.getRightTriggerAxis() > 0.5;
    double now = Timer.getFPGATimestamp();
    boolean fire = false;
    if (pressed && !lastRightTriggerPressed) {
      if (rightTriggerPressCount == 0) {
        rightTriggerPressCount = 1;
        rightTriggerFirstPressTime = now;
      } else if (rightTriggerPressCount == 1) {
        if (now - rightTriggerFirstPressTime <= 2.0) {
          fire = true;
        }
        rightTriggerPressCount = 0;
      }
    }
    if (rightTriggerPressCount == 1 && (now - rightTriggerFirstPressTime > 2.0)) {
      rightTriggerPressCount = 0;
    }
    lastRightTriggerPressed = pressed;
    return fire;
  }
	*/

  public boolean isFree(Subsystem... subsystems) {
		for (Subsystem subsystem : subsystems) {
			if (subsystem.getCurrentCommand() != null) {
				return false;
			}
		}
		return true;
	}

  public void resetState() {
		intake.resetState();
	    shooter.resetState();
		indexer.resetState();
		climb.resetState();
	}

  private void configureSysId() {
		// Set up SysId routines
		sysIdChooser.addDefaultOption("None", Commands.none());
		// Drivetrain
		sysIdChooser.addOption(
				"Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
		sysIdChooser.addOption(
				"Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
		sysIdChooser.addOption(
				"Drive SysId (Quasistatic Forward)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Drive SysId (Quasistatic Reverse)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption(
				"Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Indexer
		sysIdChooser.addOption(
				"Indexer SysId (Quasistatic Forward)", indexer.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Indexer SysId (Quasistatic Reverse)", indexer.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption(
				"Indexer SysId (Dynamic Forward)", indexer.sysIdDynamic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Indexer SysId (Dynamic Reverse)", indexer.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Shooter
		sysIdChooser.addOption(
				"Shooter SysId (Quasistatic Forward)", shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Shooter SysId (Quasistatic Reverse)", shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption(
				"Shooter SysId (Dynamic Forward)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Shooter SysId (Dynamic Reverse)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Intake
		sysIdChooser.addOption(
				"Intake SysId (Quasistatic Forward)", intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Intake SysId (Quasistatic Reverse)", intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption(
				"Intake SysId (Dynamic Forward)", intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Intake SysId (Dynamic Reverse)", intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
	}

  public Command getSysIdCommand() {
		return sysIdChooser.get();
	}

  public Command cancel() {
		return Commands.runOnce(
						() -> {
							if (drive.getCurrentCommand() != null)
								drive.getCurrentCommand().cancel();
							if (intake.getCurrentCommand() != null)
								intake.getCurrentCommand().cancel();
							if (shooter.getCurrentCommand() != null)
								shooter.getCurrentCommand().cancel();
							if (indexer.getCurrentCommand() != null)
								indexer.getCurrentCommand().cancel();
              if(climb.getCurrentCommand() != null)
                climb.getCurrentCommand().cancel();
						});
	}
}


