package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import util.Logger;

public class AutoAllign extends Command {
  private final LimelightSubsystem limelight;
  private final SwerveSubsystem drive;
  private final DoubleSupplier lateralInputMetersPerSecond;

  private final PIDController distancePID = new PIDController(0.8, 0, 0.05);
  private static final double DISTANCE_TOLERANCE = 0.1;
  private static final double TARGET_DISTANCE_METERS = 2.2;
  private static final double MAX_DRIVE_SPEED = 1.5;
  private boolean noTagFound;

  public AutoAllign(LimelightSubsystem limelight, SwerveSubsystem drive) {
    this(limelight, drive, () -> 0.0);
  }

  public AutoAllign(
      LimelightSubsystem limelight,
      SwerveSubsystem drive,
      DoubleSupplier lateralInputMetersPerSecond) {
    this.limelight = limelight;
    this.drive = drive;
    this.lateralInputMetersPerSecond = lateralInputMetersPerSecond;

    distancePID.setTolerance(DISTANCE_TOLERANCE);

    addRequirements(limelight, drive);
  }

  @Override
  public void initialize() {
    distancePID.reset();
    noTagFound = false;
    Logger.log("AutoAlign started - target distance: " + TARGET_DISTANCE_METERS + "m");
  }

  @Override
  public void execute() {
    int tagId = limelight.getTargetID();
    if (!limelight.hasValidTarget() || !isAllowedTag(tagId)) {
      drive.stop();
      if (!noTagFound) {
        Logger.log("AutoAlign: no tag found");
        noTagFound = true;
      }
      return;
    }

    double ty = limelight.getY();

    double tagHeightMeters = limelight.getTagHeightMeters(tagId);
    if (Double.isNaN(tagHeightMeters)) {
      tagHeightMeters = AutoAlignConstants.DEFAULT_TAG_HEIGHT_METERS;
    }

    double currentDistance = calculateDistanceFromTag(ty, tagHeightMeters);
    if (!Double.isFinite(currentDistance)) {
      drive.stop();
      Logger.log("AutoAlign: Invalid distance estimate... dist: "+currentDistance);
      return;
    }

    double vx = distancePID.calculate(currentDistance, TARGET_DISTANCE_METERS);
    double manualVy =
        MathUtil.clamp(lateralInputMetersPerSecond.getAsDouble(), -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    vx = MathUtil.clamp(vx, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

    double distanceError = currentDistance - TARGET_DISTANCE_METERS;
    Logger.log("Distance Error " + distanceError);

    Logger.log(
        String.format(
            "AutoAlign: tag=%d, tagZ=%.2f, target=%.2f, dist=%.2f, err=%.2f, vx=%.2f, vy=%.2f",
            tagId, tagHeightMeters, TARGET_DISTANCE_METERS, currentDistance, distanceError, vx, manualVy));

    drive.drive(vx, manualVy, 0.0);
  }


  private double calculateDistanceFromTag(double ty, double tagHeightMeters) {
    double cameraAngleDegrees = 0.0;
    double cameraHeightMeters = AutoAlignConstants.CAMERA_HEIGHT_METERS;

    double totalAngleRadians = Math.toRadians(cameraAngleDegrees + ty);
    if (Math.abs(totalAngleRadians) < 1e-3) {
      return Double.NaN;
    }

    double distance = (tagHeightMeters - cameraHeightMeters) / Math.tan(totalAngleRadians);
    return Math.abs(distance);
  }

  private boolean isAllowedTag(int tagId) {
    return tagId == 10 || tagId == 26;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.log("AutoAlign ended - interrupted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    if (noTagFound) {
      return true;
    }

    if (!limelight.hasValidTarget() || !isAllowedTag(limelight.getTargetID())) {
      return false;
    }

    boolean alignedDistance = distancePID.atSetpoint();
    Logger.log("Distance setpoint complete: " + alignedDistance);
    if (alignedDistance) {
      Logger.log("AutoAlign: Alignment complete!");
      return true;
    }
    return false;
  }
}
