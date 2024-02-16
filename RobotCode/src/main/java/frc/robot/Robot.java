// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
<<<<<<< HEAD
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vision;
=======
import frc.robot.Commands.ClimbWithoutPID;
import frc.robot.Subsystems.Climber.ClimberSide;
>>>>>>> origin/ClimberSubsystem

public class Robot extends TimedRobot implements Consts{
  private Swerve m_swerveInstance;
  private Field2d m_field;
  private Vision m_vision;
  
  @Override
  public void robotInit() {
    m_swerveInstance = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER);
    new RobotContainer();
    SmartDashboard.putNumber("max drive speed", 1);
    SmartDashboard.putNumber("max angular speed", 200);
    m_swerveInstance.zeroModulesAngles();
    //create and add robot field data to dashboard
    m_field = new Field2d();
    SmartDashboard.putData(m_field);
    m_vision = new Vision(VisionValues.LOCALIZATION_VISION_PORT);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
   // get current position and rotation of robot 
    double xCurrent = m_swerveInstance.getX();
    double yCurrent = m_swerveInstance.getY();
    double headingCurrent = m_swerveInstance.getFieldOrientedAngle();

    //update the robot position of dashboard
    m_field.setRobotPose(xCurrent, yCurrent, new Rotation2d(Math.toRadians(-headingCurrent + 90)));
  }

  @Override
  public void disabledInit() {
    m_swerveInstance.stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
<<<<<<< HEAD
    CommandScheduler.getInstance().cancelAll();
    m_swerveInstance.initSwerve();
    RobotContainer.teleop.schedule();
    
  }
  

  @Override
  public void teleopPeriodic() {
   
=======
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //test climber
    (new ClimbWithoutPID(0.5, ClimberSide.CLIMB_WITH_BOTH_SIDES)).schedule();
>>>>>>> origin/ClimberSubsystem
  }

  @Override
  public void teleopExit() {
    // m_swerveInstance.stop();
  }

  @Override
  public void testInit() {
    m_swerveInstance.zeroYaw();
    CommandScheduler.getInstance().cancelAll();
    // RobotContainer.teleop.schedule();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    // m_swerveInstance.stop();
  }
}
