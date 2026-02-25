package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

/*
THIS MAY NEED TO BE TWEAKED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/
/**
 * Drives to a specified pose.
 */
public class DriveToPose extends Command {
	private final ProfiledPIDController driveController = new ProfiledPIDController(
			Constants.AutoConstants.kPXController, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0, 3.0),
			0.02);
	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			Constants.AutoConstants.kPThetaController, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, 2*Math.PI),
			0.02);
	private Drive driveSubsystem;
	private Supplier<Pose2d> poseSupplier;
	private Translation2d lastSetpointTranslation;
	private double driveErrorAbs;
	private double thetaErrorAbs;
	private double ffMinRadius = 0.2, ffMaxRadius = 0.8;

	public DriveToPose(Drive driveSubsystem, Supplier<Pose2d> poseSupplier) {
		this.driveSubsystem = driveSubsystem;
		this.poseSupplier = poseSupplier;
		addRequirements(driveSubsystem);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		driveController.setTolerance(Meters.convertFrom(1.0, Inches));
		thetaController.setTolerance(Radians.convertFrom(0.5, Degrees));
	}

	@Override
	public void initialize() {
		Pose2d currentPose = driveSubsystem.getScorePose();
		driveController.reset(
				currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
				Math.min(
						0.0,
						-new Translation2d(driveSubsystem.getChassisSpeeds().vxMetersPerSecond,
								driveSubsystem.getChassisSpeeds().vyMetersPerSecond)
								.rotateBy(
										poseSupplier
												.get()
												.getTranslation()
												.minus(driveSubsystem
														.getScorePose()
														.getTranslation())
												.getAngle()
												.unaryMinus())
								.getX()));
		thetaController.reset(currentPose.getRotation().getRadians(),
				driveSubsystem.getChassisSpeeds().omegaRadiansPerSecond);
		lastSetpointTranslation = driveSubsystem.getScorePose().getTranslation();
	}

	@Override
	public void execute() {
		Pose2d currentPose = driveSubsystem.getScorePose();
		Pose2d targetPose = poseSupplier.get();

		Logger.recordOutput("DriveToPose/currentPose", currentPose);
		Logger.recordOutput("DriveToPose/targetPose", targetPose);

		double currentDistance = currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
		double ffScaler = MathUtil.clamp(
				(currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
				0.0,
				1.0);
		driveErrorAbs = currentDistance;
		driveController.reset(
				lastSetpointTranslation.getDistance(targetPose.getTranslation()),
				driveController.getSetpoint().velocity);
		double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
				+ driveController.calculate(driveErrorAbs, 0.0);
		if (currentDistance < driveController.getPositionTolerance())
			driveVelocityScalar = 0.0;
		lastSetpointTranslation = new Pose2d(
				targetPose.getTranslation(),
				currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
				.transformBy(new Transform2d(
						new Translation2d(driveController.getSetpoint().position, 0.0),
						new Rotation2d()))
				.getTranslation();

		// Calculate theta speed
		double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
				+ thetaController.calculate(
						currentPose.getRotation().getRadians(),
						targetPose.getRotation().getRadians());
		thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
		if (thetaErrorAbs < thetaController.getPositionTolerance())
			thetaVelocity = 0.0;

		// Command speeds
		var driveVelocity = new Pose2d(new Translation2d(),
				currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
				.transformBy(new Transform2d(
						new Translation2d(driveVelocityScalar, 0.0),
						new Rotation2d()))
				.getTranslation();
		driveSubsystem.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
				driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
	}

	@Override
	public void end(boolean interrupted) {
		driveSubsystem.stop();
	}

	@Override
	public boolean isFinished() {
		return poseSupplier.get().equals(null) || driveController.atGoal() && thetaController.atGoal();
	}
}