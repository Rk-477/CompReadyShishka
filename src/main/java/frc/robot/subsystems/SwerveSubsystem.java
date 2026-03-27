// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Meter;
public class SwerveSubsystem extends SubsystemBase {

  
  /** Creates a new ExampleSubsystem. */
File directory = new File(Filesystem.getDeployDirectory(),"swerve");
SwerveDrive  swerveDrive;

  public SwerveSubsystem() {
    try
    { 
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maximumSpeed, new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
      setupPathPlanner();
                                                                                                                                                
    } catch (Exception e){
      DriverStation.reportError("Swerve initialization failed. Check deploy/swerve config and vendor dependencies. " + e.getMessage(), e.getStackTrace());
      swerveDrive = null;
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Command to drive robot translation from left stick and heading setpoint from right stick.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY) {
    return run(() -> {
      if (swerveDrive == null) {
        return;
      }

      Translation2d scaledInputs = SwerveMath.scaleTranslation(
          new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()),
          0.8);

      driveFieldOriented(
          swerveDrive.swerveController.getTargetSpeeds(
              scaledInputs.getX(),
              scaledInputs.getY(),
              headingX.getAsDouble(),
              headingY.getAsDouble(),
              swerveDrive.getOdometryHeading().getRadians(),
              Constants.maximumSpeed));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(() -> {
      double rawRotation = angularRotationX.getAsDouble();
      double maxAngular = swerveDrive.getMaximumChassisAngularVelocity();
      double rotationCmd = rawRotation * maxAngular;
      swerveDrive.drive(
          new Translation2d(
              translationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
              translationY.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity()),
          rotationCmd,
          true,
          false);
    });
  }

  private void setupPathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          this::setChassisSpeeds,
          new PPHolonomicDriveController(
              new PIDConstants(.050, 0.0, 0.0),
              new PIDConstants(.050, 0.0, 0.0)),
          config,
          () -> DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false),
          this);
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure PathPlanner AutoBuilder: " + e.getMessage(), e.getStackTrace());
    }
  }

  public boolean isReady() {
    return swerveDrive != null;
  }

  public void driveFieldOriented (ChassisSpeeds velocity){
    if (swerveDrive != null) {
      swerveDrive.driveFieldOriented(velocity);
    }
  }
  public Command driveFieldOriented(Supplier <ChassisSpeeds> velocity) {
    return run (()-> {
      if (swerveDrive != null) {
        swerveDrive.driveFieldOriented(velocity.get());
      }
    });
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

}
