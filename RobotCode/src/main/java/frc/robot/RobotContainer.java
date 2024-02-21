// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Swerve.FollowRoute;
import frc.robot.Commands.Swerve.TurnToPoint;
import frc.robot.Commands.ClimbWithoutPID;
import frc.robot.Commands.Intake.IntakeWithoutPID;
import frc.robot.Commands.Shooter.ShootToSpeaker;
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

public class RobotContainer implements Consts{
  public RobotContainer() {
   configureBindings();
  }

  public static final XboxController chassis = new XboxController(JoystickValues.CHASSIS);
  public static final XboxController operator = new XboxController(JoystickValues.OPERATOR);
  private static final ArrayList<SwervePoint> posList = new ArrayList<SwervePoint>();

  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
      () -> chassis.getRightX(), () -> true, ChassisValues.USES_ABS_ENCODER);


  private void configureBindings() {

    // Trigger rotateRobotBy45 = new JoystickButton(chassis, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> {
    //   Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(45);
    // }));

    // Trigger rotateRobotByMinus45 = new JoystickButton(chassis, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(() -> {
    //   Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(-45);
    // }));

    Vector3d speaker = new Vector3d();
    if(DriverStation.getAlliance().get() == Alliance.Blue){
        speaker = ShooterValues.BLUE_SPAKER_POS;
    }
    else if(DriverStation.getAlliance().get() == Alliance.Red){
        speaker = ShooterValues.RED_SPAKER_POS;
    }

    Vector2d speaker2d = new Vector2d(speaker.m_x, speaker.m_z);

     Trigger r30 = new JoystickButton(chassis, XboxController.Button.kRightBumper.value).whileTrue(new ParallelCommandGroup(new InstantCommand(() -> {
        Shooter.getInstance().turnToAngle(Shooter.getInstance().getShooterAngleToSpeaker());
          }),
          new TurnToPoint(speaker2d)));

    Trigger r40 = new JoystickButton(chassis, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(() -> {
        Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);
    })).onFalse(new InstantCommand(() ->{
      Shooter.getInstance().testMotors(0);
    }));

    Trigger r45 = new JoystickButton(chassis, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> {
      Shooter.getInstance().turnToAngle(ShooterValues.AIM_MOTOR_AMP_ANGLE);
    }));
     Trigger d = new JoystickButton(chassis, XboxController.Button.kB.value).whileTrue(new InstantCommand(() -> {
      Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);
    })).onFalse(new InstantCommand(() -> {
      Shooter.getInstance().pushNoteToRollers(0);
    }));

    // Trigger r60 = new JoystickButton(chassis, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> {
    //   Shooter.getInstance().turnToAngle(60);
    // }));

    // Trigger deg = new JoystickButton(chassis, XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> {
    //   Shooter.getInstance().turnToAngle(Shooter.getInstance().getShooterAngle() + 1.0);
    // }));

    Trigger intake = new JoystickButton(chassis, XboxController.Button.kA.value).whileTrue(
      new ParallelCommandGroup(new InstantCommand(() ->{ Shooter.getInstance().turnToAngle(ShooterValues.AIM_MOTOR_MIN_ANGLE);}), new IntakeWithoutPID(IntakeValues.INTAKE_SPEED), new InstantCommand(() ->{
      Shooter.getInstance().testMotors(-0.5);
    }))).onFalse(new InstantCommand(() ->{
      Shooter.getInstance().testMotors(0);
    }));

    Trigger shootAmpTest = new JoystickButton(chassis, XboxController.Button.kY.value).whileTrue(new InstantCommand(() -> {
      Shooter.getInstance().setShootSpeed(ShooterValues.AMP_SHOOT_SPEED);
    })).onFalse(new InstantCommand(() -> {

      Shooter.getInstance().testMotors(0);
    }));



    // Trigger leftClimbUp = new JoystickButton(chassis, XboxController.Button.kA.value).whileTrue(new ClimbWithoutPID(ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_LEFT_SIDE));
    // Trigger leftClimbDown = new JoystickButton(chassis, XboxController.Button.kB.value).whileTrue(new ClimbWithoutPID(-ClimberValues.CLIMBER_SPEED, ClimberSide.
    // CLIMB_WITH_LEFT_SIDE));
    // Trigger rightClimbUp = new JoystickButton(chassis, XboxController.Button.kX.value).whileTrue(new ClimbWithoutPID(ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_RIGHT_SIDE));
    // Trigger rightClimbDown = new JoystickButton(chassis, XboxController.Button.kY.value).whileTrue(new ClimbWithoutPID(-ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_RIGHT_SIDE));
    
    
    // Trigger rotateRobot180 = new JoystickButton(chassis, XboxController.Axis.kRightTrigger.value).onTrue(new InstantCommand(() -> {
    //   Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(180);
    // }));

    // Trigger rotateRobotTo0 = new JoystickButton(chassis, XboxController.Axis.kLeftTrigger.value).onTrue(new InstantCommand(() -> {
    //   Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnToOriginOriented(0);
    //   ;
    // }));
    
    // Trigger FollowRoute = new JoystickButton(chassis, XboxController.Button.kStart.value).onTrue(
    //   new SequentialCommandGroup(new FollowRoute(posList),   new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
    //   () -> chassis.getRightX(), () -> true, ChassisValues.USES_ABS_ENCODER)));

    // Trigger savePoint = new JoystickButton(chassis, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> {
    //   posList.add(new SwervePoint(Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getX(),
    //                               Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getY(),
    //                               Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getGyro().getAngle()));
    // }));

    // Trigger removePoints = new JoystickButton(chassis, XboxController.Button.kX.value).onTrue(new InstantCommand(()->{
    //   resetPosList();
    // }));

    // Trigger resetOdometry = new JoystickButton(chassis, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> {
    //   Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).setOdometryVals(0, 0, 
    //   Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getFieldOrientedAngle());
    // })); 
    }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
  public static void resetPosList() {
    posList.clear();
  }
}
