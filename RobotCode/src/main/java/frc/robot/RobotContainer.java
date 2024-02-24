// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Swerve.FollowRoute;
import frc.robot.Commands.Swerve.TurnToPoint;
import frc.robot.Commands.ClimbWithoutPID;
import frc.robot.Commands.Intake.IntakeWithoutPID;
import frc.robot.Commands.Shooter.ShootToAmp;
import frc.robot.Commands.Shooter.ShootToSpeaker;
import frc.robot.Commands.Shooter.TurnShooterToAmp;
import frc.robot.Commands.Shooter.TurnShooterToSpeaker;
import frc.robot.Commands.Swerve.DriveByJoysticks;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Climber.ClimberSide;
import frc.robot.Utils.Consts;
import frc.robot.Utils.SwervePoint;
import frc.robot.Utils.Vector2d;
import frc.robot.Utils.Vector3d;

public class RobotContainer implements Consts {
  
  public RobotContainer() {
    configureBindings();
  }

  //controllers
  public static final CommandXboxController chassis = new CommandXboxController(JoystickValues.CHASSIS);
  public static final CommandXboxController operator = new CommandXboxController(JoystickValues.OPERATOR);

  //command values
  private static boolean m_driveMode = true;
  private static final ArrayList<SwervePoint> posList = new ArrayList<SwervePoint>();
  private static Vector2d m_speaker2d = new Vector2d(0,0);
  
  //command instances
  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
      () -> chassis.getRightX(), () -> m_driveMode, ChassisValues.USES_ABS_ENCODER);

  private void configureBindings() {
    
    //Chassis driver buttons

    //turn chassis
    chassis.rightBumper().onTrue(new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(90);}));
    chassis.leftBumper().onTrue(new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(-90);}));
    chassis.rightStick().onTrue(new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnToOriginOriented(0);}));
    //turn chassis to speaker
    chassis.a().whileTrue(new SequentialCommandGroup(
      new InstantCommand(() -> {
        Vector3d speaker3d = new Vector3d();
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
          speaker3d = ShooterValues.BLUE_SPAKER_POS;
        } 
        else if (DriverStation.getAlliance().get() == Alliance.Red) {
          speaker3d = ShooterValues.RED_SPAKER_POS;
        }
      m_speaker2d = new Vector2d(speaker3d.m_x, speaker3d.m_z);
      }),

      new TurnToPoint(m_speaker2d))).onFalse(teleop);
    //switch between origin and robot oriented
    chassis.start().onTrue(teleop).onTrue(new InstantCommand(() -> {m_driveMode = !m_driveMode;}));
    //change drive speeds
    chassis.rightTrigger().whileTrue(new InstantCommand(() -> {SmartDashboard.putNumber("max drive speed", ChassisValues.FAST_MODE_DRIVE_SPEED);})).
                          onFalse(new InstantCommand(() -> {SmartDashboard.putNumber("max drive speed", ChassisValues.DRIVE_GEAR_RATIO);}));
    
    chassis.leftTrigger().whileTrue(new InstantCommand(() -> {SmartDashboard.putNumber("max drive speed", ChassisValues.SLOW_MODE_DRIVE_SPEED);})).
                          onFalse(new InstantCommand(() -> {SmartDashboard.putNumber("max drive speed", ChassisValues.DRIVE_GEAR_RATIO);}));
    
    //Operator buttons

    //intake
    operator.a().toggleOnTrue(
          new InstantCommand(() -> {
            Shooter.getInstance().turnToAngle(ShooterValues.AIM_MOTOR_MIN_ANGLE);
            Intake.getInstance().intakeWithoutPID(IntakeValues.INTAKE_SPEED);
            Shooter.getInstance().pullNoteWithoutPID(-0.5);
          }))          
        .toggleOnFalse(
          new InstantCommand(() ->{
            Shooter.getInstance().pullNoteWithoutPID(0);
            Intake.getInstance().intakeWithoutPID(0);
          })
        );
    
    //amp
    operator.x().toggleOnTrue(new ParallelCommandGroup(new TurnShooterToAmp(), new ShootToAmp()))
                .toggleOnFalse(new InstantCommand(() -> {Shooter.getInstance().shoot(0);}));

    //speaker
    operator.y().toggleOnTrue(new ParallelCommandGroup(new TurnShooterToSpeaker(), new InstantCommand(() -> {Shooter.getInstance().shoot(ShooterValues.SPEAKER_SHOOT_SPEED);})))
                .toggleOnFalse(new InstantCommand(() -> {Shooter.getInstance().shoot(0);}));

    //push note to rollers
    operator.b()
        .whileTrue(new InstantCommand(() -> {
          Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);
        })).onFalse(new InstantCommand(() -> {
          Shooter.getInstance().pushNoteToRollers(0);
    }));    
    
    //climb
    operator.rightBumper().whileTrue(new ClimbWithoutPID(ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_RIGHT_SIDE));
    operator.leftBumper().whileTrue(new ClimbWithoutPID(ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_LEFT_SIDE));
    operator.rightTrigger().whileTrue(new ClimbWithoutPID(-ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_RIGHT_SIDE));
    operator.leftTrigger().whileTrue(new ClimbWithoutPID(-ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_LEFT_SIDE));


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static void resetPosList() {
    posList.clear();
  }
}
