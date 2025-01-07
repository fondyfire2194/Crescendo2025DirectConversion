// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.Pref;

public class TransferSubsystem extends SubsystemBase {

  public SparkMax transferMotor;
  public SparkClosedLoopController transferController;
  SparkMaxConfig transferConfig;

  private AngularVelocity commandrpm=RPM.of(0);

  public SparkLimitSwitch m_limitSwitch;

  public boolean simnoteatintake;
  public boolean skipFirstNoteInSim;
  public boolean skipSecondNoteInSim;
  public boolean skipThirdNoteInSim;
  public boolean skipFourthNoteInSim;

  public boolean lobbing;

  public boolean shootmoving;
  public boolean transferMotorConnected;

  public boolean OKShootMoving;

  Debouncer noteDetector = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  /** Creates a new transfer. */
  public TransferSubsystem() {

    transferMotor = new SparkMax(CANIDConstants.transferID, MotorType.kBrushless);
    transferController = transferMotor.getClosedLoopController();
    m_limitSwitch = transferMotor.getForwardLimitSwitch();
    transferConfig = new SparkMaxConfig();

    transferConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    transferConfig.encoder        
        .positionConversionFactor(TransferConstants.transferConversionPositionFactor)
        .velocityConversionFactor(TransferConstants.transferConversionVelocityFactor);

    transferConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(TransferConstants.transferKFF)
        .pid(TransferConstants.transferKp, TransferConstants.transferKi, TransferConstants.transferKd);

    transferConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    transferConfig.signals.primaryEncoderPositionPeriodMs(5);

    transferMotor.configure(transferConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void stopMotor() {
    runAtVelocity(0);
    transferMotor.stopMotor();
    commandrpm = RPM.of(0);
  }

  public Command stopTransferCommand() {
    commandrpm = RPM.of(0);
    return Commands.runOnce(() -> stopMotor(), this);
  }

  public Command transferToShooterCommand() {
    return Commands.run(() -> transferToShooter())
        .withTimeout(TransferConstants.clearShooterTime)
        .andThen(stopTransferCommand());
  }

  public Command transferToShooterCommandAmp() {
    return Commands.run(() -> transferToShooterAmp())
        .until(() -> !noteAtIntake())
        .withTimeout(TransferConstants.clearShooterTime)
        .andThen(stopTransferCommand());
  }

  public void transferToShooterAmp() {
    enableLimitSwitch(false);
    commandrpm =RPM.of (Pref.getPref("AmpTransferToShootSpeed"));
    simnoteatintake = false;
    runAtVelocity(commandrpm.in(RPM));
  }

  public void transferToShooter() {
    enableLimitSwitch(false);
    commandrpm = TransferConstants.transferToShootSpeed;// Pref.getPref("TransferToShootSpeed");
    runAtVelocity(commandrpm.in(RPM));
    simnoteatintake = false;
  }

  public void runToSensor() {
    enableLimitSwitch(true);

    // commandrpm=Pref.getPref("TransferIntakingSpeed");
    commandrpm = TransferConstants.intakingSpeed;
    runAtVelocity(commandrpm.in(RPM));
  }

  public void runAtVelocity(double rpm) {
    if (RobotBase.isReal())
      transferController.setReference(rpm, ControlType.kVelocity);
  }

  public boolean noteAtIntake() {
    return m_limitSwitch.isPressed();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!transferMotorConnected) {
      transferMotorConnected = checkMotorCanOK(transferMotor);
      SmartDashboard.putBoolean("Transfer//OKTransferMotor", transferMotorConnected);
    }
  }

  private boolean checkMotorCanOK(SparkMax motor) {

    return RobotBase.isSimulation();
  }

  public Command testCan() {
    return Commands.runOnce(() -> transferMotorConnected = false);
  }

  public void enableLimitSwitch(boolean enable) {
    transferConfig.limitSwitch.forwardLimitSwitchEnabled(enable);
  }

  public boolean getLimitSwitchEnabled() {
    return transferMotor.configAccessor.limitSwitch.getForwardLimitSwitchEnabled();
  }

  public double getAmps() {
    return transferMotor.getOutputCurrent();
  }

  public double getRPM() {
    if (RobotBase.isReal())
      return transferMotor.getEncoder().getVelocity();
    else
      return commandrpm.in(RPM);
  }

  public boolean onPlusHardwareLimit() {
    return transferMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean onMinusHardwareLimit() {
    return transferMotor.getReverseLimitSwitch().isPressed();
  }

  public boolean getActiveFault() {
    return transferMotor.hasActiveFault();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> transferMotor.clearFaults());
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

  public double getPosition() {
    return transferMotor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return transferMotor.getEncoder().getVelocity();
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < Pref.getPref("TransferIntakingSpeed") / 20;
  }

}