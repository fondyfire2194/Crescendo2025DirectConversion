package frc.robot.subsystems;

import org.w3c.dom.ls.LSParser;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Pref;

public class SwerveModule extends SubsystemBase {
  public int moduleNumber;
  private Rotation2d lastAngle = new Rotation2d();

  private SparkMax angleMotor;
  SparkMax driveMotor;

  private final CANcoder m_turnCancoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;
  private SparkMaxConfig driveConfig;
  private SparkMaxConfig turnConfig;

  private SimpleMotorFeedforward driveFeedforward;

  private SwerveModuleState currentDesiredState = new SwerveModuleState();
  private double m_simDrivePosition;
  private double m_simRotatePosition;
  private boolean driveReversed;
  private double characterizationVolts;
  private boolean characterizing;
  private SwerveModuleState previousState = new SwerveModuleState();
  public boolean wheelAligning;
  private double feedForward;
  private double acceleration;
  private double ffks;
  private double ffka;
  private double ffkv;
  private double velocityError;
  private boolean tuning;
  public double angleatabsolutereset;
  private double rotationalAcceleration;

  public double testAngle;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {

    driveConfig = new SparkMaxConfig();
    turnConfig = new SparkMaxConfig();

    this.moduleNumber = moduleNumber;
    driveReversed = moduleConstants.driveReversed;
    /* Angle Motor Config */
    angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    /* Drive Motor Config */
    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);

    angleController = angleMotor.getClosedLoopController();

    driveConfig
        .idleMode(IdleMode.kBrake)
        .inverted(driveReversed);
    driveConfig.encoder
        .positionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor)
        .velocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
    driveConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(ShooterConstants.topShooterKFF)
        .outputRange(0, 1)
        .pid(Constants.SwerveConstants.driveKP, Constants.SwerveConstants.driveKI, Constants.SwerveConstants.driveKD);
    driveConfig.signals.primaryEncoderPositionPeriodMs(5);
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnConfig
        .idleMode(IdleMode.kBrake)
        .inverted(driveReversed);
    turnConfig.encoder
        .positionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
    turnConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

        .outputRange(0, 1)
        .positionWrappingMaxInput(180)
        .positionWrappingMinInput(-180)
        .positionWrappingEnabled(true)
        .pidf(Constants.SwerveConstants.angleKP[moduleNumber],
            Constants.SwerveConstants.angleKI[moduleNumber],
            Constants.SwerveConstants.angleKD[moduleNumber],
            Constants.SwerveConstants.angleKFF[moduleNumber]);
    turnConfig.signals.primaryEncoderPositionPeriodMs(5);
    angleMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.m_simDrivePosition = 0.0;
    this.m_simRotatePosition = 0.0;
    m_turnCancoder = new CANcoder(moduleConstants.canCoderID, "CV1");

    driveController = driveMotor.getClosedLoopController();

    driveFeedforward = new SimpleMotorFeedforward(
        SwerveConstants.driveKS[moduleNumber],
        SwerveConstants.driveKV[moduleNumber],
        SwerveConstants.driveKA[moduleNumber]);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getAngle());

    currentDesiredState = optimizedState;
    // https://www.chiefdelphi.com/t/yet-another-generic-swerve-library-yagsl-beta/425148/860

    if (SwerveConstants.useCosineCompensation) {
      double cosineScalar = optimizedState.angle.minus(getAngle()).getCos();
      if (cosineScalar < 0.0) {
        cosineScalar = 1.0;
      }
      optimizedState.speedMetersPerSecond *= cosineScalar;
    }
    setAngle(optimizedState);

    setSpeed(optimizedState, isOpenLoop);
  }

  public SwerveModuleState getDesiredState() {
    return currentDesiredState;
  }

  public boolean resetAngleToAbsolute() {
    angleatabsolutereset = 0;
    String tcn = m_turnCancoder.getNetwork();
    if (RobotBase.isReal()) {
      angleatabsolutereset = (m_turnCancoder.getAbsolutePosition().getValueAsDouble() * 360);
    }
    angleMotor.getEncoder().setPosition(angleatabsolutereset);
    return tcn == "CV1";
  }

  public void resetAngleEncoder(double angle) {
    driveMotor.getEncoder().setPosition(angle);
    this.m_simRotatePosition = 0.0;
  }

  public void resetDriveEncoder() {
    driveMotor.getEncoder().setPosition(0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double percentOutput = 0;
    if (isOpenLoop) {
      percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.kmaxSpeed;
      driveMotor.setVoltage(percentOutput * RobotController.getBatteryVoltage());
    }
    // showModuleValues(percentOutput);
    boolean feedforward = true;

    if (!isOpenLoop) {
      if (!feedforward) {
        driveController.setReference(desiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
      } else {
        double temp = previousState.speedMetersPerSecond;
        if (temp == 0)
          temp = desiredState.speedMetersPerSecond;

        acceleration = (desiredState.speedMetersPerSecond - temp) / 0.020;
        acceleration = Math.min(Math.max(acceleration, -5), 5);
        tuning = false;

        if (!tuning)
          feedForward = driveFeedforward.calculate(
              desiredState.speedMetersPerSecond, acceleration);
        else
          feedForward = calculate(desiredState.speedMetersPerSecond, acceleration);

        feedForward = MathUtil.clamp(feedForward, -12.0, 12.0);

        if (Math.abs(desiredState.speedMetersPerSecond) < .01) {
          feedForward = 0;
        }
        driveController.setReference(
            desiredState.speedMetersPerSecond, ControlType.kVelocity);

        velocityError = desiredState.speedMetersPerSecond - getDriveVelocity();

        previousState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
      }
    }
    if (RobotBase.isSimulation())
      m_simDrivePosition += desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_PERIOD;
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = !wheelAligning
        && (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.kmaxSpeed * 0.001))
            ? lastAngle
            : desiredState.angle;
    String a = "Modules//" + String.valueOf(moduleNumber);
    SmartDashboard.putNumber(a + " DesAngle", desiredState.angle.getDegrees());
    angleController.setReference(angle.getDegrees(), ControlType.kPosition);

    rotationalAcceleration = angle.getDegrees() - lastAngle.getDegrees() / .02;
    lastAngle = angle;
    if (RobotBase.isSimulation())
      m_simRotatePosition = angle.getDegrees();
  }

  private Rotation2d getAngle() {
    if (RobotBase.isReal())
      return Rotation2d.fromDegrees(
          MathUtil.inputModulus(driveMotor.getEncoder().getPosition(), -180, 180));
    else
      return Rotation2d.fromDegrees(MathUtil.inputModulus(m_simRotatePosition, -180, 180));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getAngle());
  }

  public double getVoltage() {
    return driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
  }

  public double getDriveVelocity() {
    if (RobotBase.isReal())
      return driveMotor.getEncoder().getVelocity();
    else {
      return currentDesiredState.speedMetersPerSecond;
    }
  }

  public double getDriveKv() {
    return (getVoltage() - getAcceleration() - SwerveConstants.driveKS[moduleNumber]) / getDriveVelocity();
  }

  public double getDrivePosition() {
    if (RobotBase.isReal())
      return driveMotor.getEncoder().getPosition();
    else {
      return m_simDrivePosition;
    }
  }

  public double getDriveCurrent() {
    if (RobotBase.isReal())
      return driveMotor.getOutputCurrent();
    else {
      return 0;
    }
  }

  public double getTurnCurrent() {
    if (RobotBase.isReal())
      return angleMotor.getOutputCurrent();
    else {
      return 0;
    }
  }

  public double getAcceleration() {
    return acceleration;
  }

  public double getRotationalAcceleration() {
    return rotationalAcceleration;
  }

  public double getFeedforward() {
    return feedForward;
  }

  public double getPositionRadians() {
    return getDrivePosition() / SwerveConstants.wheelRadius;
  }

  public boolean getModuleAngleAt0() {
    return Math.abs(getAngle().getDegrees()) < .25;
  }

  public double getVelocityError() {
    return velocityError;
  }

  @Override
  public void periodic() {
    String a = "Modules//" + String.valueOf(moduleNumber);

    SmartDashboard.putNumber(a + " DrivePosition", getDrivePosition());
    SmartDashboard.putNumber(a + " CanCoderAngle", getCancoderDeg());
    SmartDashboard.putNumber(a + " TurnAngle", Units.radiansToDegrees(getPositionRadians()));

    if (tuning) {
      SmartDashboard.putNumber(a + " VelocityError", velocityError);
      SmartDashboard.putNumber(a + " feedforward", feedForward);
      SmartDashboard.putNumber(a + " acceleration", acceleration);
      SmartDashboard.putNumber(a + " ffks", ffks);
      SmartDashboard.putNumber(a + " ffka", ffka);
      SmartDashboard.putNumber(a + " ffkv", ffkv);

    }

    if (characterizing) {
      driveMotor.setVoltage(characterizationVolts);
      angleController.setReference(0, ControlType.kPosition);
      SmartDashboard.putBoolean(a + " Characterizing",
          characterizing);
      SmartDashboard.putNumber(a + " Characterization Volts", characterizationVolts);
    }

  }

  public double getCancoderDeg() {
    return m_turnCancoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  public boolean isStopped() {
    return Math.abs(getDriveVelocity()) < .1;
  }

  public void setIdleMode(boolean brake) {
    if (brake) {
      driveConfig.idleMode(IdleMode.kBrake);
      turnConfig.idleMode(IdleMode.kBrake);
    } else {
      driveConfig.idleMode(IdleMode.kCoast);
      turnConfig.idleMode(IdleMode.kCoast);
    }
  }

  public Command clearFaultsCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> driveMotor.clearFaults()),
        runOnce(() -> angleMotor.clearFaults()),
        runOnce(() -> m_turnCancoder.clearStickyFaults()));
  }

  public void setCharacterizationVolts(double volts) {
    characterizationVolts = volts;
    characterizing = true;
  }

  public void stopCharacterizing() {
    characterizationVolts = 0.0;
    characterizing = false;
  }

  @Override
  public void simulationPeriodic() {

  }

  public double calculate(double velocity, double acceleration) {
    ffks = Math.signum(velocity) * Pref.getPref("driveKs");
    double temp = SmartDashboard.getNumber("kv" + SwerveConstants.modNames[moduleNumber], 2.4);
    ffkv = velocity * temp;
    if (acceleration > 0)
      ffka = acceleration * Pref.getPref("driveKa");
    else
      ffka = acceleration * Pref.getPref("driveKadown");
    return ffks + ffka + ffkv;
  }

  private void showModuleValues(double percentOutput) {
    SmartDashboard.putNumber("Modules/" + String.valueOf(moduleNumber) + " driveVolts",
        percentOutput * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Modules/" + String.valueOf(moduleNumber) + " driveVelocity",
        getDriveVelocity());

    double temp1 = (percentOutput * RobotController.getBatteryVoltage())
        - SwerveConstants.driveKS[moduleNumber] / getDriveVelocity();
    SmartDashboard.putNumber("Modules/" + String.valueOf(moduleNumber) + " drivevolts",
        temp1);
    double drkv = temp1 / getDriveVelocity();
    SmartDashboard.putNumber("Modules/" + String.valueOf(moduleNumber) + " drivekv",
        drkv);

    SmartDashboard.putBoolean("Modules/" + String.valueOf(moduleNumber) + " braked",
        driveMotor.configAccessor.getIdleMode() == IdleMode.kBrake);
  }

}
