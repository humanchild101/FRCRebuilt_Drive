package frc.robot.commands.factories;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ScorePositionsUtil;
import frc.robot.util.ScorePositionsUtil.ScoringSector;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveFactory {
    public static Command driveToClimbAlignmentPosition(Drive drive, Supplier<ScoringSector> sectorSupplier) {
        return DriveCommands.driveToPose(
                drive,
                () -> ScorePositionsUtil.getPoseInAlliance(
                        ScorePositionsUtil.climbPositions.get(sectorSupplier.get()).align(),
                        DriverStation.getAlliance().orElse(Alliance.Blue)))
                .withName("DriveToClimbAlignmentPosition");
    }

    public static Command driveToHumanFuelAlignmentPosition(Drive drive, Supplier<ScoringSector> sectorSupplier) {
        return DriveCommands.driveToPose(
                drive,
                () -> ScorePositionsUtil.getPoseInAlliance(
                        ScorePositionsUtil.humanFuelPositions.get(sectorSupplier.get()).align(),
                        DriverStation.getAlliance().orElse(Alliance.Blue)))
                .withName("DriveToHumanFuelAlignmentPosition");
    }
/* 
    public static Command autoAlignmentPosition(Drive drive, Supplier<ScoringSector> sectorSupplier) {
        return DriveCommands.driveToPose(
                drive,
                () -> ScorePositionsUtil.getPoseInAlliance(
                        ScorePositionsUtil.autoPositions.get(sectorSupplier.get()).align(),
                        DriverStation.getAlliance().orElse(Alliance.Blue)))
                .withName("DriveToAutoAlignmentPosition");
    }
*/
}
