package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import util.Logger;

public class AutoAllign extends Command {
  private final LimelightSubsystem limelight;
  private final SwerveSubsystem drive;

  private final PIDController distancePID = new PIDController(0.8, 0, 0.05);
  private final PIDController strafePID = new PIDController(0.05, 0, 0.005);
  private final PIDController rotationPID = new PIDController(0.03, 0, 0.002);

  private final double defaultTargetDistance;

  private static final double DISTANCE_TOLERANCE = 0.1;
  private static final double STRAFE_TOLERANCE = 2.0;
  private static final double ROTATION_TOLERANCE = 3.0;

  private static final double MAX_DRIVE_SPEED = 1.5;
  private static final double MAX_ROTATION_SPEED = 2.0;

  public AutoAllign(LimelightSubsystem limelight, SwerveSubsystem drive) {
    this(limelight, drive, AutoAlignConstants.DEFAULT_TARGET_DISTANCE_METERS);
  }

  public AutoAllign(
      LimelightSubsystem limelight, SwerveSubsystem drive, double defaultTargetDistanceMeters) {
    this.limelight = limelight;
    this.drive = drive;
    this.defaultTargetDistance = defaultTargetDistanceMeters;

    distancePID.setTolerance(DISTANCE_TOLERANCE);
    strafePID.setTolerance(STRAFE_TOLERANCE);
    rotationPID.setTolerance(ROTATION_TOLERANCE);

    addRequirements(limelight, drive);
  }

  @Override
  public void initialize() {
    distancePID.reset();
    strafePID.reset();
    rotationPID.reset();
    Logger.log("AutoAlign started - default target distance: " + defaultTargetDistance + "m");
  }

  @Override
  public void execute() {
    if (!limelight.hasValidTarget()) {
      drive.stop();
      Logger.log("AutoAlign: No valid target detected");
      return;
    }

    int tagId = limelight.getTargetID();
    if (!isAllowedTag(tagId)) {
      drive.stop();
      Logger.log("AutoAlign: Ignoring unsupported tag ID " + tagId);
      return;
    }

    double tx = limelight.getX();
    double ty = limelight.getY();

    double tagHeightMeters = limelight.getTagHeightMeters(tagId);
    if (Double.isNaN(tagHeightMeters)) {
      tagHeightMeters = AutoAlignConstants.DEFAULT_TAG_HEIGHT_METERS;
    }

    double targetDistance = getTargetDistanceForTag(tagId);
    double currentDistance = calculateDistanceFromTag(ty, tagHeightMeters, targetDistance);

    double distanceError = currentDistance - targetDistance;
    double strafeError = tx;
    double rotationError = tx;

    double vx = -distancePID.calculate(distanceError, 0);
    double vy = -strafePID.calculate(strafeError, 0);
    double omega = -rotationPID.calculate(rotationError, 0);

    vx = MathUtil.clamp(vx, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    vy = MathUtil.clamp(vy, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    omega = MathUtil.clamp(omega, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    //error log
    Logger.log("Distance Error "+ distanceError);

    Logger.log(
        String.format(
            "AutoAlign: tag=%d, tagZ=%.2f, target=%.2f, dist=%.2f, err=%.2f, tx=%.1f, vx=%.2f, vy=%.2f, omega=%.2f",
            tagId, tagHeightMeters, targetDistance, currentDistance, distanceError, tx, vx, vy, omega));

    drive.drive(vx, vy, omega);
  }


  private double calculateDistanceFromTag(
      double ty, double tagHeightMeters, double fallbackDistanceMeters) {
    double cameraAngleDegrees = 0.0;
    double cameraHeightMeters = AutoAlignConstants.CAMERA_HEIGHT_METERS;

    double totalAngleRadians = Math.toRadians(cameraAngleDegrees + ty);
    if (Math.abs(totalAngleRadians) < 0.01) {
      return fallbackDistanceMeters;
    }

    double distance = (tagHeightMeters - cameraHeightMeters) / Math.tan(totalAngleRadians);
    return Math.abs(distance);
  }

  private double getTargetDistanceForTag(int tagId) {
    return AutoAlignConstants.TARGET_DISTANCE_METERS_BY_TAG.getOrDefault(
        tagId, defaultTargetDistance);
  }

  private boolean isAllowedTag(int tagId) {
    return AutoAlignConstants.TARGET_DISTANCE_METERS_BY_TAG.containsKey(tagId);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.log("AutoAlign ended - interrupted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    if (!limelight.hasValidTarget()) {
      return false;
    }

    int tagId = limelight.getTargetID();
    if (!isAllowedTag(tagId)) {
      return false;
    }

    boolean aligned = distancePID.atSetpoint() && strafePID.atSetpoint() && rotationPID.atSetpoint();
    if (aligned) {
      Logger.log("AutoAlign: Alignment complete!");
    }
    return aligned;
  }
}
