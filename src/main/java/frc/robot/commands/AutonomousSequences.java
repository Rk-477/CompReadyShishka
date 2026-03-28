package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public final class AutonomousSequences {
  private static final double AUTO_BACKUP_SPEED = -0.28;
  private static final double AUTO_BACKUP_TIME_SEC = 1.35;
  private static final double AUTO_SMALL_TURN_SPEED = 0.20;
  private static final double AUTO_SMALL_TURN_TIME_SEC = 0.10;
  private static final double AUTO_SHOOT_TIME_SEC = 3.0;

  private AutonomousSequences() {
    throw new UnsupportedOperationException("Utility class");
  }

  public static Command middle(SwerveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
    return Commands.sequence(
        driveRobotRelativeFor(driveSubsystem, AUTO_BACKUP_SPEED, 0.0, 0.0, AUTO_BACKUP_TIME_SEC),
        Commands.waitSeconds(0.10),
        shootFor(shooterSubsystem, AUTO_SHOOT_TIME_SEC),
        Commands.runOnce(driveSubsystem::stop, driveSubsystem));
  }

  public static Command left(SwerveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
    return Commands.sequence(
        driveRobotRelativeFor(driveSubsystem, AUTO_BACKUP_SPEED, 0.0, 0.0, AUTO_BACKUP_TIME_SEC),
        driveRobotRelativeFor(driveSubsystem, 0.0, 0.0, -AUTO_SMALL_TURN_SPEED, AUTO_SMALL_TURN_TIME_SEC),
        shootFor(shooterSubsystem, AUTO_SHOOT_TIME_SEC),
        Commands.runOnce(driveSubsystem::stop, driveSubsystem));
  }

  public static Command right(SwerveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
    return Commands.sequence(
        driveRobotRelativeFor(driveSubsystem, AUTO_BACKUP_SPEED, 0.0, 0.0, AUTO_BACKUP_TIME_SEC),
        driveRobotRelativeFor(driveSubsystem, 0.0, 0.0, AUTO_SMALL_TURN_SPEED, AUTO_SMALL_TURN_TIME_SEC),
        shootFor(shooterSubsystem, AUTO_SHOOT_TIME_SEC),
        Commands.runOnce(driveSubsystem::stop, driveSubsystem));
  }

  private static Command driveRobotRelativeFor(
      SwerveSubsystem driveSubsystem,
      double xSpeed,
      double ySpeed,
      double rotSpeed,
      double seconds) {
    return Commands.run(
            () -> driveSubsystem.drive(xSpeed, ySpeed, rotSpeed),
            driveSubsystem)
        .withTimeout(seconds);
  }

  private static Command shootFor(ShooterSubsystem shooterSubsystem, double seconds) {
    return Commands.startEnd(
            () -> shooterSubsystem.setShooterPower(0.85),
            shooterSubsystem::stop,
            shooterSubsystem)
        .withTimeout(seconds);
  }
}
