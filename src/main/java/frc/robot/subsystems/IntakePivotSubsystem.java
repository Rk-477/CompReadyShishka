package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import util.Logger;

public class IntakePivotSubsystem extends SubsystemBase {
    private static final int PIVOT_CAN_ID = 14;

    // Time-based calibration values (seconds).
    private static final double PIVOT_DOWN_DURATION_SEC = 0.5;
    private static final double PIVOT_UP_DURATION_SEC = 0.5;
    private static final double PIVOT_POWER_DOWN = 0.30;
    private static final double PIVOT_POWER_UP = -0.20;
    private static final double PIVOT_POWER_HOLD_DOWN = 0.08;
    private static final int PIVOT_CURRENT_LIMIT = 40;

    private final SparkMax pivotMotor;

    private boolean movingDown = true;
    private boolean enabled = false;
    private boolean lastEnabled = false;
    private double moveStartTimeSec = 0.0;
    private double moveDurationSec = 0.0;

    public IntakePivotSubsystem() {
        pivotMotor = new SparkMax(PIVOT_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(PIVOT_CURRENT_LIMIT)
            .openLoopRampRate(0.05);

        pivotMotor.configure(
            pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        stop();
        Logger.log("Intake pivot subsystem initialized on CAN 14");
    }

    private void startMove(boolean shouldMoveDown, double durationSec) {
        movingDown = shouldMoveDown;
        moveDurationSec = durationSec;
        moveStartTimeSec = Timer.getFPGATimestamp();
        enabled = true;

        Logger.log(
            String.format(
                "Intake pivot move requested. Direction=%s Duration=%.2fs",
                movingDown ? "DOWN" : "UP",
                moveDurationSec
            )
        );
        Logger.log(
            String.format(
                "Intake pivot status on toggle. BusV=%.2f Applied=%.2f OutputCurrent=%.2f Enabled=%s",
                pivotMotor.getBusVoltage(),
                pivotMotor.getAppliedOutput(),
                pivotMotor.getOutputCurrent(),
                enabled
            )
        );
    }

    public void moveDown() {
        startMove(true, PIVOT_DOWN_DURATION_SEC);
    }

    public void moveDownFor(double durationSec) {
        startMove(true, durationSec);
    }

    public void moveUp() {
        startMove(false, PIVOT_UP_DURATION_SEC);
    }

    public void moveUpFor(double durationSec) {
        startMove(false, durationSec);
    }

    /**
     * Apply a low continuous down-hold power to resist intake lift without high stress.
     * Intended for while-held button behavior.
     */
    public void holdDownSoft() {
        enabled = false;
        pivotMotor.set(MathUtil.clamp(PIVOT_POWER_HOLD_DOWN, -1.0, 1.0));
    }

    public void stop() {
        pivotMotor.stopMotor();
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void periodic() {
        double nowSec = Timer.getFPGATimestamp();
        double elapsedSec = nowSec - moveStartTimeSec;

        SmartDashboard.putBoolean("IntakePivot/Moving", enabled);
        SmartDashboard.putBoolean("IntakePivot/MovingDown", movingDown);
        SmartDashboard.putNumber("IntakePivot/MoveDurationSec", moveDurationSec);
        SmartDashboard.putNumber("IntakePivot/ElapsedSec", enabled ? elapsedSec : 0.0);
        SmartDashboard.putNumber("IntakePivot/AppliedOutput", pivotMotor.getAppliedOutput());
        if (!enabled) {
            lastEnabled = false;
            return;
        }

        if (!lastEnabled) {
            Logger.log(
                String.format(
                    "Intake pivot enabled. Direction=%s Duration=%.2fs BusV=%.2f Applied=%.2f OutputCurrent=%.2f",
                    movingDown ? "DOWN" : "UP",
                    moveDurationSec,
                    pivotMotor.getBusVoltage(),
                    pivotMotor.getAppliedOutput(),
                    pivotMotor.getOutputCurrent()
                )
            );
            lastEnabled = true;
        }

        if (elapsedSec >= moveDurationSec) {
            stop();
            return;
        }

        pivotMotor.set(MathUtil.clamp(movingDown ? PIVOT_POWER_DOWN : PIVOT_POWER_UP, -1.0, 1.0));
    }
}
