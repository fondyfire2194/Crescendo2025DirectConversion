// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LLPipelines;
import frc.robot.utils.LimelightHelpers;


public class Robot extends TimedRobot  {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  private double m_startDelay;

  private double startTime;

  private boolean autoHasRun;

  int tstctr;

  private int canivorectr;



  @Override
  public void robotInit() {
    startTime = Timer.getFPGATimestamp();

    if (RobotBase.isReal()) {
      DriverStation.startDataLog(DataLogManager.getLog());

      // Map<Integer, String> motorNameMap = new HashMap<>();

      // motorNameMap.put(SwerveConstants.Mod0.driveMotorID, "Front Left Drive");
      // motorNameMap.put(SwerveConstants.Mod0.angleMotorID, "Front Left Turn");

      // motorNameMap.put(SwerveConstants.Mod1.driveMotorID, "Front Right Drive");
      // motorNameMap.put(SwerveConstants.Mod1.angleMotorID, "Front Right Turn");

      // motorNameMap.put(SwerveConstants.Mod2.driveMotorID, "Back Left Drive");
      // motorNameMap.put(SwerveConstants.Mod2.angleMotorID, "Back Left Turn");

      // motorNameMap.put(SwerveConstants.Mod3.driveMotorID, "Back Right Drive");
      // motorNameMap.put(SwerveConstants.Mod3.angleMotorID, "Back Right Turn");

      // motorNameMap.put(CANIDConstants.armID, "Arm");

      // motorNameMap.put(CANIDConstants.transferID, "Transfer");

      // motorNameMap.put(CANIDConstants.topShooterID, "Shooter Top");
      // motorNameMap.put(CANIDConstants.bottomShooterID, "Shooter Bottom");

      // motorNameMap.put(CANIDConstants.intakeID, "Intake");

      // motorNameMap.put(CANIDConstants.climberIDLeft, "Climber Left");
      // motorNameMap.put(CANIDConstants.climberIDRight, "Climber Right");

      // URCL.start(motorNameMap);
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);

    }

    m_robotContainer = new RobotContainer();
    if (RobotBase.isSimulation()) {
      SmartDashboard.putBoolean("Skip1", m_robotContainer.m_transfer.skipFirstNoteInSim);
      SmartDashboard.putBoolean("Skip2", m_robotContainer.m_transfer.skipSecondNoteInSim);
      SmartDashboard.putBoolean("Skip3", m_robotContainer.m_transfer.skipThirdNoteInSim);
      SmartDashboard.putBoolean("Skip4", m_robotContainer.m_transfer.skipFourthNoteInSim);

    }


    DriverStation.startDataLog(DataLogManager.getLog());


  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    

    if (RobotBase.isReal()) {

      canivorectr++;
      if (canivorectr >= 100) {
      
        canivorectr = 0;
      }
    
    }

    
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    autoHasRun = false;

  
    m_robotContainer.m_arm.enableArm = false;
    // if (m_robotContainer.m_arm.getMotorDegrees() < 45)
    //   m_robotContainer.m_arm.armMotor.configure

   

  }

  @Override
  public void disabledPeriodic() {

    // turn off drive brakes if they are on and robot is not moving
    // allows easier manual pushing of robot

    // if (m_robotContainer.m_swerve.driveIsBraked() && m_robotContainer.m_swerve.isStopped()
    //     && m_disableStartTime == 0)
    //   m_disableStartTime = Timer.getFPGATimestamp();

    // if (m_disableStartTime != 0 && Timer.getFPGATimestamp() > m_disableStartTime
    //     + brakeOffTime) {
    //   m_robotContainer.m_swerve.setIdleMode(false);
    // }
    // if (Timer.getFPGATimestamp() > startupTimeSeconds + 5)
    //   m_robotContainer.checkAutoSelectLoop.poll();

  }

  @Override
  public void disabledExit() {

  }

  @Override
  public void autonomousInit() {
    

  }

  @Override
  public void autonomousPeriodic() {
    if (!autoHasRun && Timer.getFPGATimestamp() > startTime + m_startDelay
        && m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      autoHasRun = true;
    }

  }

  @Override
  public void autonomousExit() {
    m_robotContainer.m_shooter.stopMotors();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_shooter.stopMotors();
    m_robotContainer.m_intake.stopMotor();
    m_robotContainer.m_transfer.stopMotor();

  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    

    m_robotContainer.m_arm.setGoal(m_robotContainer.m_arm.getAngle());

    m_robotContainer.m_shooter.stopMotors();
    m_robotContainer.m_intake.stopMotor();
    m_robotContainer.m_transfer.stopMotor();

   // m_robotContainer.setDefaultCommands();
    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.frontRightCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname,
        LLPipelines.pipelines.NOTEDET1.ordinal());
   
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
   
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
   
    double a1 = AllianceUtil.flipFieldAngle(FieldConstants.centerNotesPickup[1]).getRotation().getDegrees();
    double a2 = AllianceUtil.flipFieldAngle(FieldConstants.centerNotesPickup[2]).getRotation().getDegrees();

    SmartDashboard.putNumber("A1", a1);
    SmartDashboard.putNumber("A2", a2);

  }
}
