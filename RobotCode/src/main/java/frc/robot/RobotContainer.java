// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Swerve.TurnToPoint;
import frc.robot.Commands.Swerve.TurnToSpeaker;
import frc.robot.Commands.Swerve.TurnToSpeakerAuto;
import frc.robot.Commands.ClimbWithoutPID;
import frc.robot.Commands.Intake.IntakeWithoutPID;
import frc.robot.Commands.Shooter.ShootToAmp;
import frc.robot.Commands.Shooter.TurnShooterToAmp;
import frc.robot.Commands.Shooter.TurnShooterToSpeaker;
import frc.robot.Commands.Swerve.DriveByJoysticks;
import frc.robot.Commands.Swerve.FollowRoute;
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
  public static final XboxController operatorRumble = new XboxController(JoystickValues.OPERATOR);

  //command values
  private static final ArrayList<SwervePoint> posList = new ArrayList<SwervePoint>();
  
  
  //command instances
  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
      () -> chassis.getRightX(), () -> true, ChassisValues.USES_ABS_ENCODER);

    private void configureBindings() {
    
    //Chassis driver buttons
    
    //turn chassis
    // chassis.rightBumper().onTrue(new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(90);}));
    // chassis.leftBumper().onTrue(new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(-90);}));

    //turn chassis to speaker
    chassis.a().whileTrue(new TurnToSpeaker()).onFalse(teleop);

    chassis.start().onTrue(new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).zeroYaw();}));

    //change drive speeds
    chassis.rightTrigger().whileTrue(new InstantCommand(() -> {SmartDashboard.putNumber("max drive speed", ChassisValues.FAST_MODE_DRIVE_SPEED);})).
                          onFalse(new InstantCommand(() -> {SmartDashboard.putNumber("max drive speed", ChassisValues.DRIVE_SPEED);}));
    
    chassis.leftTrigger().whileTrue(new InstantCommand(() -> {SmartDashboard.putNumber("max drive speed", ChassisValues.SLOW_MODE_DRIVE_SPEED);})).
                          onFalse(new InstantCommand(() -> {SmartDashboard.putNumber("max drive speed", ChassisValues.DRIVE_SPEED);}));
    

    //Operator buttons

    //intake
    operator.a().whileTrue(
          new IntakeWithoutPID(IntakeValues.INTAKE_SPEED)
          )          
        .onFalse(
          new InstantCommand(() ->{
            Shooter.getInstance().pullNoteWithoutPID(0);
            Intake.getInstance().intakeWithoutPID(0);
          })
        );

    // reverse intake
    operator.back().whileTrue(
          new IntakeWithoutPID(-IntakeValues.INTAKE_SPEED)
          )          
        .onFalse(
          new InstantCommand(() ->{
            Shooter.getInstance().pullNoteWithoutPID(0);
            Intake.getInstance().intakeWithoutPID(0);
          })
        );
    
    //amp
    operator.x().whileTrue(new ParallelCommandGroup(new TurnShooterToAmp(), new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(0, 600);})))
                .onFalse(new InstantCommand(() -> {Shooter.getInstance().stopRollers();}));

    //speaker
    operator.y().whileTrue(new ParallelCommandGroup(new TurnShooterToSpeaker(), new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);})))
                .onFalse(new InstantCommand(() -> {Shooter.getInstance().stopRollers();}));

    //push note to rollers
    operator.b()
        .whileTrue(new InstantCommand(() -> {
          Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);
        })).onFalse(new InstantCommand(() -> {
          Shooter.getInstance().pushNoteToRollers(0);
    }));    

    //speaker from close
    //center
    operator.povUp().whileTrue(new ParallelCommandGroup(new InstantCommand(() -> {Shooter.getInstance().turnToAngle(114);}), new InstantCommand(()->{Shooter.getInstance().setShootSpeed(7000, ShooterValues.SPEAKER_SHOOT_SPEED * 1.1 / 3);})))
                .onFalse(new InstantCommand(() -> {Shooter.getInstance().stopRollers();}));
    
    //amp side 
    operator.povLeft().whileTrue(new ParallelCommandGroup(new InstantCommand(() -> {Shooter.getInstance().turnToAngle(122);}), new InstantCommand(()->{Shooter.getInstance().setShootSpeed(7000, 5000);})))
                .onFalse(new InstantCommand(() -> {Shooter.getInstance().stopRollers();}));
    
    //not amp side
    operator.povRight().whileTrue(new ParallelCommandGroup(new InstantCommand(() -> {Shooter.getInstance().turnToAngle(60);}), new InstantCommand(()->{Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED, ShooterValues.SPEAKER_SHOOT_SPEED);})))
                .onFalse(new InstantCommand(() -> {Shooter.getInstance().stopRollers();}));
    
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
