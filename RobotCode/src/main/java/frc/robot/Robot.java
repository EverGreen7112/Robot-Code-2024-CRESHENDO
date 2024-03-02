// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Autonomous.DriveAndShoot;
import frc.robot.Commands.Autonomous.TwoNoteAndPickupAmpSideAuto;
import frc.robot.Commands.Autonomous.TwoNoteAuto;
import frc.robot.Commands.Shooter.TurnShooterToSpeaker;
import frc.robot.Commands.Swerve.DriveByJoysticks;
import frc.robot.Commands.Swerve.TurnToSpeaker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.JetsonHealthChecker;
import frc.robot.Utils.Vector2d;
import frc.robot.Utils.Vector3d;
import frc.robot.Utils.LocalizationVision;


public class Robot extends TimedRobot implements Consts{
  private Swerve m_swerveInstance;
  private Field2d m_field;
  private LocalizationVision m_vision;
  private JetsonHealthChecker m_jetsonHealthChecker;
  private static SendableChooser<Alliance> m_allianceChooser = new SendableChooser<Alliance>();;
  
  @Override
  public void robotInit() {
    new RobotContainer();
    Shooter.getInstance();
    //init drive speeds
    SmartDashboard.putNumber("max drive speed", 1);
    SmartDashboard.putNumber("max angular speed",200);
    //init swerve
    m_swerveInstance = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER);
    //create and add robot field data to dashboard
    m_field = new Field2d();
    SmartDashboard.putData(m_field);
    //start vision
    m_vision = new LocalizationVision(VisionValues.LOCALIZATION_VISION_PORT);
    m_jetsonHealthChecker = new JetsonHealthChecker(VisionValues.JETSON_HEALTH_CHECK_PORT);
    //add alliance options to dashboard
    m_allianceChooser.setDefaultOption("red", Alliance.Red);
    m_allianceChooser.addOption("blue", Alliance.Blue);
    SmartDashboard.putData("alliance chooser", m_allianceChooser);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //get current position and rotation of robot 
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
    m_swerveInstance.initSwerve();
    new TwoNoteAndPickupAmpSideAuto().schedule();
    // new DriveAndShoot().schedule();
    // .andThen(new ParallelCommandGroup(new TurnToSpeaker(), new TurnShooterToSpeaker())).schedule();;

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    m_swerveInstance.initSwerve();
    RobotContainer.teleop.schedule();
  }
  

  @Override
  public void teleopPeriodic() {

      //rumble when shooter is ready to shoot
      if(Shooter.getInstance().isReadyToShoot()){
        RobotContainer.operatorRumble.setRumble(RumbleType.kBothRumble, 1);
      }
      else {
        RobotContainer.operatorRumble.setRumble(RumbleType.kBothRumble, 0);
      }
  }

  @Override
  public void teleopExit() {
    m_swerveInstance.stop();
    CommandScheduler.getInstance().cancelAll();
  }


  public static Alliance getAlliance(){
    return m_allianceChooser.getSelected();
  }
}
