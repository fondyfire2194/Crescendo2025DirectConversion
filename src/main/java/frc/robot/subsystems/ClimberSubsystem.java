// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */
  SparkMax climberMotorLeft;
  SparkMax climberMotorRight;

  SparkMaxConfig leftConfig;
  SparkMaxConfig rightConfig;

  SparkMaxSim climberLeftSim;
  SparkMaxSim climberRightSim;

  public boolean leftMotorConnected;
  public boolean rightMotorConnected;

  public double simpositionleft;

  private double simpositionright;

  public double positionError;
  public boolean directionMinus;

  public ClimberSubsystem() {
    climberMotorLeft = new SparkMax(CANIDConstants.climberIDLeft, MotorType.kBrushless);
    leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(false);
    leftConfig.secondaryCurrentLimit(Constants.ClimberConstants.climberContinuousCurrentLimit);
    leftConfig.encoder.positionConversionFactor(Constants.ClimberConstants.climberConversionPositionFactor);
    leftConfig.encoder.velocityConversionFactor(Constants.ClimberConstants.climberConversionVelocityFactor);
    leftConfig.voltageCompensation(Constants.ClimberConstants.voltageComp);
    leftConfig.openLoopRampRate(1);

    climberMotorLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberMotorRight = new SparkMax(CANIDConstants.climberIDRight, MotorType.kBrushless);

    rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.inverted(false);
    rightConfig.secondaryCurrentLimit(Constants.ClimberConstants.climberContinuousCurrentLimit);
    rightConfig.encoder.positionConversionFactor(Constants.ClimberConstants.climberConversionPositionFactor);
    rightConfig.encoder.velocityConversionFactor(Constants.ClimberConstants.climberConversionVelocityFactor);
    rightConfig.voltageCompensation(Constants.ClimberConstants.voltageComp);
    rightConfig.openLoopRampRate(1);

    climberMotorLeft.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberLeftSim = new SparkMaxSim(climberMotorLeft, DCMotor.getNeo550(1));
    climberRightSim = new SparkMaxSim(climberMotorRight, DCMotor.getNeo550(1));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!leftMotorConnected) {
      leftMotorConnected = checkMotorCanOK(climberMotorLeft);
      SmartDashboard.putBoolean("Climber//OKLClimber", leftMotorConnected);
    }

    if (!rightMotorConnected) {
      rightMotorConnected = checkMotorCanOK(climberMotorRight);
      SmartDashboard.putBoolean("Climber//OKRClimber", rightMotorConnected);
    }
  }

  private boolean checkMotorCanOK(SparkMax motor) {
    // double temp = motor.getOpenLoopRampRate();
    // return RobotBase.isSimulation() || motor.setOpenLoopRampRate(temp) ==
    // REVLibError.kOk;
    return true;
  }

  public Command testCan() {
    return Commands.parallel(
        Commands.runOnce(() -> leftMotorConnected = false),
        runOnce(() -> rightMotorConnected = false));
  }

  public void stopMotors() {
    stopLeftMotor();
    stopRightMotor();
  }

  public void stopRightMotor() {
    climberMotorRight.stopMotor();
    climberMotorRight.setVoltage(0);
  }

  public void stopLeftMotor() {
    climberMotorLeft.stopMotor();
    climberMotorLeft.setVoltage(0);
  }

  public Command stopClimberCommand() {
    return Commands.parallel(
        stopLeftClimberCommand(),
        stopRightClimberCommand());
  }

  public Command stopLeftClimberCommand() {
    return Commands.runOnce(() -> stopLeftMotor());
  }

  public Command stopRightClimberCommand() {
    return Commands.runOnce(() -> stopRightMotor());
  }

  public void runLeftClimberMotor(double speed) {
    climberMotorLeft.setVoltage(speed * RobotController.getBatteryVoltage());

  }

  public void runRightClimberMotor(double speed) {
    climberMotorRight.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public void runClimberMotors(double speed) {
    runRightClimberMotor(speed);
    runLeftClimberMotor(speed);
  }

  public void raiseClimber(double speed) {
    if (getPositionLeft() > ClimberConstants.maxPosition - 3)
      speed = .05;

    if (getPositionLeft() >= ClimberConstants.maxPosition)
      stopMotors();
    else
      runClimberMotors(speed);
  }

  public void lowerClimber(double speed) {
    speed *= -1;
    if (getPositionLeft() < ClimberConstants.minPosition + 3)
      speed = -.05;

    if (getPositionLeft() < ClimberConstants.minPosition + 1)
      stopMotors();
    else
      runClimberMotors(speed);
  }

  public Command lowerClimberArmsCommand(double speed) {
    return Commands.run(() -> lowerClimber(speed));
  }

  public Command raiseClimberArmsCommand(double speed) {
    SmartDashboard.putNumber("Climber/upspeed2", speed);
    return Commands.run(() -> raiseClimber(speed));
  }

  public double getRPMLeft() {
    return climberMotorLeft.getEncoder().getVelocity();
  }

  public double getRPMRight() {
    return climberMotorLeft.getEncoder().getVelocity();
  }

  public double getPositionLeft() {
    if (RobotBase.isReal())
      return climberMotorLeft.getEncoder().getPosition();
    else
      return simpositionleft;
  }

  public double getPositionRight() {
    if (RobotBase.isReal())
      return climberMotorRight.getEncoder().getPosition();
    else
      return simpositionright;
  }

  public boolean getLeftAtTarget(double target) {
    return !directionMinus && getPositionLeft() > target || directionMinus && getPositionLeft() < target;
  }

  public boolean getRightAtTarget(double target) {
    return !directionMinus && getPositionRight() > target || directionMinus && getPositionRight() < target;
  }

  // public int getLeftStickyFaults() {
  // return climberMotorLeft.getStickyFaults();
  // }

  // public int getRightStickyFaults() {
  // return climberMotorRight.getStickyFaults();
  // }

  public Command clearStickyFaultsCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> climberMotorLeft.clearFaults()),
        runOnce(() -> climberMotorRight.clearFaults()));
  }

  public double getLeftAmps() {
    return climberMotorLeft.getOutputCurrent();
  }

  public double getRightAmps() {
    return climberMotorRight.getOutputCurrent();
  }

  @Override
  public void simulationPeriodic() {

  }
}