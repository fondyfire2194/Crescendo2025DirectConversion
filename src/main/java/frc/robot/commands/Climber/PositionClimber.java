// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class PositionClimber extends Command {
  /** Creates a new JogShooter. */
  private ClimberSubsystem m_climber;
  private double m_target;

  public PositionClimber(ClimberSubsystem climber, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_target = target;

  }

  // Called when the command is initially scheduled.
  @Override
  // public void initialize() {
  //   m_climber.directionMinus = false;
  // }
  public void initialize() {
    m_climber.directionMinus = m_target<m_climber.getPositionLeft();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Climber/target", m_target);
    m_climber.positionError = m_target - m_climber.getPositionLeft();

    double usespeed = .3;
    if (m_climber.positionError < 0) {
      usespeed = -.3;
      m_climber.directionMinus = true;
    }
    SmartDashboard.putNumber("Climber/poserr", m_climber.positionError);
    if (Math.abs(m_climber.positionError) < 6)
      usespeed *= .5;

    if (Math.abs(m_climber.positionError) < 1)
      usespeed *= .25;
    SmartDashboard.putNumber("Climber/usespd", usespeed);
    if (!m_climber.getLeftAtTarget(m_target)) {
      m_climber.runLeftClimberMotor(usespeed);
      m_climber.runRightClimberMotor(usespeed);
    } else {
      m_climber.stopLeftMotor();
      m_climber.stopRightMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.getLeftAtTarget(m_target)
        || m_climber.getPositionLeft() > ClimberConstants.maxPosition
        || m_climber.getPositionLeft() < ClimberConstants.minPosition;

  }
}
