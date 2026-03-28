// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAllign;
import frc.robot.commands.AutonomousSequences;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
    // Shooter left/right, tower, conveyor, intake roller CAN IDs from shooter-pid branch.
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(11, 9, 12, 13, 16);
    private final AutoAllign autoAllignCommand = new AutoAllign(limelightSubsystem, drivebase);
    private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();
    private boolean invertDriveBindings = false;
  
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // Configure the trigger bindings
      configureBindings();
      configureAutonomousChooser();
      if (drivebase.isReady()) {
        SwerveInputStream driveDirectAngle = SwerveInputStream
            .of(
                drivebase.getSwerveDrive(),
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX())
            .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
            .headingWhile(true)
            .deadband(Constants.OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

        // Keep this command prepared for later switching.
        Command driveFieldOrientedSwerveInput = drivebase.driveFieldOriented(driveDirectAngle);

        // Applies deadbands and uses right stick as desired heading setpoint.
        Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
            () -> applyDriveInversion(MathUtil.applyDeadband(
                m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND)),
            () -> applyDriveInversion(MathUtil.applyDeadband(
                m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)),
            () -> applyDriveInversion(m_driverController.getRightX()));

        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
      }

      // Feed robot pose to Limelight simulation support.
      limelightSubsystem.setRobotPoseSupplier(drivebase::getPose);
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Run AutoAlign on A.
    m_driverController.a().onTrue(autoAllignCommand);
    m_operatorController.a().onTrue(autoAllignCommand);

    // Cancel AutoAlign on left trigger.
    m_driverController.leftTrigger().onTrue(Commands.runOnce(autoAllignCommand::cancel));
    m_operatorController.leftTrigger().onTrue(Commands.runOnce(autoAllignCommand::cancel));

    // Toggle shooter+tower+conveyor on B (operator controller).
    m_operatorController.b().toggleOnTrue(Commands.startEnd(
        () -> shooterSubsystem.shootWithPID(),
        () -> shooterSubsystem.stop(),
        shooterSubsystem));

    // Toggle intake roller on X (operator controller).
    m_operatorController.x().onTrue(Commands.runOnce(
        () -> shooterSubsystem.toggleIntakeOnly(0.6),
        shooterSubsystem));
    
    // Toggle manual drive inversion on Y for alliance-side perspective changes.
    m_driverController.y().onTrue(Commands.runOnce(() -> {
      invertDriveBindings = !invertDriveBindings;
      SmartDashboard.putBoolean("Drive Controls Inverted", invertDriveBindings);
    }));

    // Move intake pivot down on right trigger (operator controller).
    m_operatorController.rightTrigger().onTrue(Commands.runOnce(
        intakePivotSubsystem::moveDown,
        intakePivotSubsystem));

    // Gently hold intake pivot down while right bumper is held.
    m_operatorController.rightBumper().whileTrue(Commands.run(
        intakePivotSubsystem::holdDownSoft,
        intakePivotSubsystem));
    m_operatorController.rightBumper().onFalse(Commands.runOnce(
        intakePivotSubsystem::stop,
        intakePivotSubsystem));

    // Move intake pivot up on left bumper (operator controller).
    m_operatorController.leftBumper().onTrue(Commands.runOnce(
        intakePivotSubsystem::moveUp,
        intakePivotSubsystem));
  }

  private void configureAutonomousChooser() {
    autonomousChooser.setDefaultOption("Middle", AutonomousSequences.middle(drivebase, shooterSubsystem));
    autonomousChooser.addOption("Left", AutonomousSequences.left(drivebase, shooterSubsystem));
    autonomousChooser.addOption("Right", AutonomousSequences.right(drivebase, shooterSubsystem));
    SmartDashboard.putBoolean("Drive Controls Inverted", invertDriveBindings);
    SmartDashboard.putData("Autonomous", autonomousChooser);
  }

  private double applyDriveInversion(double value) {
    return invertDriveBindings ? -value : value;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
