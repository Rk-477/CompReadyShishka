package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import java.util.function.Supplier;

public class LimelightSubsystem extends SubsystemBase {
  private final NetworkTable limelightTable;
  private final AprilTagFieldLayout aprilTagField =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  private double millisTimeRecorded;

  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
  private final NetworkTableEntry tv;

  private final Field2d visionField = new Field2d();
  private final StructPublisher<Pose3d> visionPose3dPublisher;
  private final StructPublisher<Pose3d> tagPosePublisher;

  private Supplier<Pose2d> robotPoseSupplier;
  private int simTagId;
  private double simTx;
  private double simTy;
  private boolean simHasTarget;
  private static final double SIM_MAX_DISTANCE = 5.0;
  private static final double SIM_FOV_HORIZONTAL = 27.0;

  public LimelightSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
    tv = limelightTable.getEntry("tv");

    NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    visionPose3dPublisher = visionTable.getStructTopic("VisionPose3d", Pose3d.struct).publish();
    tagPosePublisher = visionTable.getStructTopic("DetectedTagPose", Pose3d.struct).publish();

    SmartDashboard.putData("VisionField", visionField);
    setPipeline(0);
  }

  @Override
  public void periodic() {
    double currentX = tx.getDouble(0.0);
    double currentY = ty.getDouble(0.0);
    double currentArea = ta.getDouble(0.0);
    double currentTarget = tv.getDouble(0.0);
    millisTimeRecorded = WPIUtilJNI.now() * 1e-3;

    SmartDashboard.putNumber("Limelight X", currentX);
    SmartDashboard.putNumber("Limelight Y", currentY);
    SmartDashboard.putNumber("Limelight Area", currentArea);
    SmartDashboard.putBoolean("Limelight Has Target", currentTarget > 0.5);
    SmartDashboard.putNumber("Limelight distance", getDistanceFromTag(1.6, getY()));
    SmartDashboard.putNumber("Limelight Tag ID", getTargetID());

    Pose3d botPose = getBotPose3d();
    if (botPose != null) {
      Rotation3d rotation = botPose.getRotation();
      SmartDashboard.putBoolean("Limelight Botpose Valid", true);
      SmartDashboard.putNumber("Limelight Roll", Math.toDegrees(rotation.getX()));
      SmartDashboard.putNumber("Limelight Pitch", Math.toDegrees(rotation.getY()));
      SmartDashboard.putNumber("Limelight Yaw", Math.toDegrees(rotation.getZ()));
    } else {
      SmartDashboard.putBoolean("Limelight Botpose Valid", false);
      SmartDashboard.putNumber("Limelight Roll", Double.NaN);
      SmartDashboard.putNumber("Limelight Pitch", Double.NaN);
      SmartDashboard.putNumber("Limelight Yaw", Double.NaN);
    }

    if (isTargetValid()) {
      int tagId = getTargetID();
      Pose3d tagPose = aprilTagField.getTagPose(tagId).orElse(null);
      if (tagPose != null) {
        tagPosePublisher.set(tagPose);
        SmartDashboard.putNumber("Detected Tag ID", tagId);
      }
    }
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double getY() {
    return ty.getDouble(0.0);
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }

  public boolean hasValidTarget() {
    return getRawTargetValue() > 0.5;
  }

  public void setLEDMode(int mode) {
    limelightTable.getEntry("ledMode").setNumber(mode);
  }

  public void setCameraMode(int mode) {
    limelightTable.getEntry("camMode").setNumber(mode);
  }

  public void setPipeline(int pipeline) {
    limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void setExposure(double exposure) {
    limelightTable.getEntry("exposure").setNumber(exposure);
  }

  public void setBlackLevel(double blackLevel) {
    limelightTable.getEntry("black_level").setNumber(blackLevel);
  }

  public double getRawTargetValue() {
    return limelightTable.getEntry("tv").getDouble(0.0);
  }

  public int getTargetID() {
    return (int) limelightTable.getEntry("tid").getDouble(0.0);
  }

  public double getTagHeightMeters(int tagId) {
    if (tagId <= 0) {
      return Double.NaN;
    }
    Pose3d tagPose = aprilTagField.getTagPose(tagId).orElse(null);
    if (tagPose == null) {
      return Double.NaN;
    }
    return tagPose.getZ();
  }

  public boolean isTargetValid() {
    return limelightTable.getEntry("tv").getDouble(0.0) == 1;
  }

  public double getTimeRecordedInMilis() {
    return millisTimeRecorded;
  }

  public double getDistanceFromTag(double tagHeight, double tagYDegrees) {
    double heightDiff = tagHeight - LimelightConstants.kCameraToRobot.getZ();
    double angleRadians = Math.toRadians(tagYDegrees);
    if (Math.abs(angleRadians) < 1e-3) {
      return Double.NaN;
    }
    return Math.abs(heightDiff / Math.tan(angleRadians));
  }
  

  public synchronized Pose2d getPose(Rotation2d robotRotation2d) {
    if (!isTargetValid()) {
      return null;
    }

    Pose3d tag = aprilTagField.getTagPose(getTargetID()).orElse(null);
    if (tag == null) {
      return null;
    }

    double yaw = robotRotation2d.getRadians();
    double distance = getDistanceFromTag(tag.getZ(), getY());
    if (!Double.isFinite(distance)) {
      return null;
    }
    double beta = yaw - Math.toRadians(getY());
    double x = Math.cos(beta) * distance;
    double y = Math.sin(beta) * distance;
    Translation2d tagToCamera = new Translation2d(-x, -y);

    Pose2d cameraPose = new Pose2d(tag.toPose2d().getTranslation().plus(tagToCamera), new Rotation2d(yaw));
    Translation2d offset = LimelightConstants.kCameraToRobot.toTranslation2d().rotateBy(robotRotation2d);
    Pose2d robotPose = new Pose2d(cameraPose.getTranslation().minus(offset), new Rotation2d(yaw));

    visionField.setRobotPose(robotPose);
    visionPose3dPublisher.set(new Pose3d(robotPose));
    return robotPose;
  }

  public Pose3d getBotPose3d() {
    double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    if (botpose.length < 6) {
      return null;
    }

    Pose3d pose =
        new Pose3d(
            botpose[0],
            botpose[1],
            botpose[2],
            new Rotation3d(
                Math.toRadians(botpose[3]),
                Math.toRadians(botpose[4]),
                Math.toRadians(botpose[5])));
    visionPose3dPublisher.set(pose);
    visionField.setRobotPose(pose.toPose2d());
    return pose;
  }

  public void updateVisionPose(Rotation2d robotRotation) {
    Pose3d botPose = getBotPose3d();
    if (botPose != null) {
      return;
    }
    getPose(robotRotation);
  }

  public void setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
    robotPoseSupplier = poseSupplier;
  }

  @Override
  public void simulationPeriodic() {
    if (robotPoseSupplier == null) {
      return;
    }
    Pose2d robotPose = robotPoseSupplier.get();
    if (robotPose == null) {
      return;
    }

    simHasTarget = false;
    simTagId = 0;
    double closestDistance = Double.MAX_VALUE;
    Pose3d closestTag = null;

    for (int tagId = 1; tagId <= 22; tagId++) {
      var tagPoseOpt = aprilTagField.getTagPose(tagId);
      if (tagPoseOpt.isEmpty()) {
        continue;
      }

      Pose3d tagPose = tagPoseOpt.get();
      Pose2d tagPose2d = tagPose.toPose2d();
      double distance = robotPose.getTranslation().getDistance(tagPose2d.getTranslation());
      if (distance > SIM_MAX_DISTANCE) {
        continue;
      }

      Translation2d robotToTag = tagPose2d.getTranslation().minus(robotPose.getTranslation());
      double angleToTag = Math.toDegrees(Math.atan2(robotToTag.getY(), robotToTag.getX()));
      double robotHeading = robotPose.getRotation().getDegrees();
      double relativeAngle = angleToTag - robotHeading;
      while (relativeAngle > 180) {
        relativeAngle -= 360;
      }
      while (relativeAngle < -180) {
        relativeAngle += 360;
      }
      if (Math.abs(relativeAngle) > SIM_FOV_HORIZONTAL) {
        continue;
      }

      if (distance < closestDistance) {
        closestDistance = distance;
        simTagId = tagId;
        simTx = relativeAngle;
        simTy =
            Math.toDegrees(
                Math.atan2(tagPose.getZ() - LimelightConstants.kCameraToRobot.getZ(), distance));
        simHasTarget = true;
        closestTag = tagPose;
      }
    }

    if (simHasTarget) {
      limelightTable.getEntry("tv").setDouble(1.0);
      limelightTable.getEntry("tid").setDouble(simTagId);
      limelightTable.getEntry("tx").setDouble(simTx);
      limelightTable.getEntry("ty").setDouble(simTy);
      limelightTable
          .getEntry("ta")
          .setDouble(100.0 / (closestDistance * closestDistance));

      double[] botpose = {
        robotPose.getX(),
        robotPose.getY(),
        0.0,
        0.0,
        0.0,
        robotPose.getRotation().getDegrees()
      };
      limelightTable.getEntry("botpose_wpiblue").setDoubleArray(botpose);

      Pose3d visionPose = new Pose3d(robotPose);
      visionPose3dPublisher.set(visionPose);
      visionField.setRobotPose(robotPose);
      if (closestTag != null) {
        tagPosePublisher.set(closestTag);
      }
    } else {
      limelightTable.getEntry("tv").setDouble(0.0);
      limelightTable.getEntry("tid").setDouble(0.0);
    }
  }
}
