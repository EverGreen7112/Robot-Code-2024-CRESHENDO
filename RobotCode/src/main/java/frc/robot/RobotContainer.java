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

public class RobotContainer implements Consts {
  
  public RobotContainer() {
    configureBindings();
  }

  public static boolean driveMode = true;
  public static final XboxController chassis = new XboxController(JoystickValues.CHASSIS);
  public static final XboxController operator = new XboxController(JoystickValues.OPERATOR);
  private static final ArrayList<SwervePoint> posList = new ArrayList<SwervePoint>();
  private static Vector2d m_speaker2d = new Vector2d(0,0);
  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
      () -> chassis.getRightX(), () -> driveMode, ChassisValues.USES_ABS_ENCODER);

  private void configureBindings() {


    //chassis driver buttons

    Trigger turnRobotBy90 = new JoystickButton(chassis,
      XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(90);
    }));

    Trigger turnRobotByMinus90 = new JoystickButton(chassis,
      XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(-90);
    }));
    
    Trigger turnRobotTo0 = new JoystickButton(chassis,
      XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnToOriginOriented(0);
    }));

    Trigger changeDriveMode = new JoystickButton(chassis,
      XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> {
      driveMode = !driveMode;
    })); 
    
    Trigger fastMode = new JoystickButton(chassis, XboxController.Axis.kRightTrigger.value).onTrue(new InstantCommand(() ->{ 
      ChassisValues.MAX_SPEED = new Supplier<Double>() {
        @Override
        public Double get() {
          return ChassisValues.FAST_MODE_DRIVE_SPEED;
        }
      };
    })).onFalse(new InstantCommand(() -> {
      ChassisValues.MAX_SPEED = new Supplier<Double>() {
        @Override
        public Double get() {
          return ChassisValues.DRIVE_SPEED;
        }
      };
    }));

    Trigger slowMode = new JoystickButton(chassis, XboxController.Axis.kLeftTrigger.value).onTrue(new InstantCommand(() ->{ 
      ChassisValues.MAX_SPEED = new Supplier<Double>() {
        @Override
        public Double get() {
          return ChassisValues.SLOW_MODE_DRIVE_SPEED;
        }
      };
    })).onFalse(new InstantCommand(() -> {
      ChassisValues.MAX_SPEED = new Supplier<Double>() {
        @Override
        public Double get() {
          return ChassisValues.DRIVE_SPEED;
        }
      };
    }));

     Vector3d speaker = new Vector3d();
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      speaker = ShooterValues.BLUE_SPAKER_POS;
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
      speaker = ShooterValues.RED_SPAKER_POS;
    }
    m_speaker2d = new Vector2d(speaker.m_x, speaker.m_z);
    
    Trigger turnRobotToSpeaker = new JoystickButton(chassis, XboxController.Button.kB.value).whileTrue(new SequentialCommandGroup(
      new InstantCommand(() -> {
      Vector3d speaker3d = new Vector3d();
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        speaker3d = ShooterValues.BLUE_SPAKER_POS;
      } else if (DriverStation.getAlliance().get() == Alliance.Red) {
        speaker3d = ShooterValues.RED_SPAKER_POS;
      }
      m_speaker2d = new Vector2d(speaker3d.m_x, speaker3d.m_z);
      }), 
      new TurnToPoint(m_speaker2d))).onFalse(teleop);

   Trigger intake = new JoystickButton(chassis, XboxController.Button.kA.value)
        .toggleOnTrue(
          new InstantCommand(() -> {
            Shooter.getInstance().turnToAngle(ShooterValues.AIM_MOTOR_MIN_ANGLE);
            Intake.getInstance().intakeWithoutPID(IntakeValues.INTAKE_SPEED);
            Shooter.getInstance().testMotors(-0.5);
          }))          
        .toggleOnFalse(
          new InstantCommand(() ->{
            Shooter.getInstance().testMotors(0);
            Intake.getInstance().intakeWithoutPID(0);
          })
        );

      
    //operator buttons
      Trigger turnToSpeaker = new JoystickButton(operator, XboxController.Button.kA.value)
        .whileTrue(new InstantCommand(() -> {
          Shooter.getInstance().turnToAngle(Shooter.getInstance().getShooterAngleToSpeaker());
          Shooter.getInstance().shoot(ShooterValues.SPEAKER_SHOOT_SPEED);
        }
        )).onFalse(new InstantCommand(() -> {
          Shooter.getInstance().shoot(0);
        }));

  
    Trigger shootToAmp = new JoystickButton(operator, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> {
            Shooter.getInstance().turnToAngle(ShooterValues.AIM_MOTOR_AMP_ANGLE);
            Shooter.getInstance().shoot(ShooterValues.AMP_SHOOT_SPEED);

    }));


    Trigger pushNoteToRollers = new JoystickButton(operator, XboxController.Button.kB.value)
        .whileTrue(new InstantCommand(() -> {
          Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);
        })).onFalse(new InstantCommand(() -> {
          Shooter.getInstance().pushNoteToRollers(0);
    }));

    // Trigger leftClimbUp = new JoystickButton(operator,
    //     XboxController.Button.kLeftBumper.value)
    //     .whileTrue(new ClimbWithoutPID(ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_LEFT_SIDE));

    // Trigger leftClimbDown = new JoystickButton(operator,
    //     XboxController.Axis.kLeftTrigger.value)
    //     .whileTrue(new ClimbWithoutPID(-ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_LEFT_SIDE));

    // Trigger rightClimbUp = new JoystickButton(operator,
    //     XboxController.Button.kRightBumper.value)
    //     .whileTrue(new ClimbWithoutPID(ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_RIGHT_SIDE));

    // Trigger rightClimbDown = new JoystickButton(operator,
    //     XboxController.Axis.kRightTrigger.value)
    //     .whileTrue(new ClimbWithoutPID(-ClimberValues.CLIMBER_SPEED, ClimberSide.CLIMB_WITH_RIGHT_SIDE));
    

    // Trigger FollowRoute = new JoystickButton(chassis,
    // XboxController.Button.kStart.value).onTrue(
    // new SequentialCommandGroup(new FollowRoute(posList), new DriveByJoysticks(()
    // -> chassis.getLeftX(), () -> chassis.getLeftY(),
    // () -> chassis.getRightX(), () -> true, ChassisValues.USES_ABS_ENCODER)));

    // Trigger savePoint = new JoystickButton(chassis,
    // XboxController.Button.kA.value).onTrue(new InstantCommand(() -> {
    // posList.add(new
    // SwervePoint(Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getX(),
    // Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getY(),
    // Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getGyro().getAngle()));
    // }));

    // Trigger removePoints = new JoystickButton(chassis,
    // XboxController.Button.kX.value).onTrue(new InstantCommand(()->{
    // resetPosList();
    // }));

    // Trigger resetOdometry = new JoystickButton(chassis,
    // XboxController.Button.kB.value).onTrue(new InstantCommand(() -> {
    // Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).setOdometryVals(0,
    // 0,
    // Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getFieldOrientedAngle());
    // }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static void resetPosList() {
    posList.clear();
  }
}
