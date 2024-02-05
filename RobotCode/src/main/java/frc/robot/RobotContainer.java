// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Swerve.FollowRoute;
import frc.robot.Commands.Swerve.DriveByJoysticks;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.SwervePoint;

public class RobotContainer implements Consts{
  public RobotContainer() {
    configureBindings();
  }

  public static final XboxController chassis = new XboxController(JoystickValues.CHASSIS);
  public static final XboxController operator = new XboxController(JoystickValues.OPERATOR);

  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
      () -> chassis.getRightX(), () -> true, ChassisValues.USES_ABS_ENCODER);
  

  private void configureBindings() {

    Trigger rotateRobotBy45 = new JoystickButton(chassis, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(45);
    }));

    Trigger rotateRobotByMinus45 = new JoystickButton(chassis, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(-45);
      ;
    }));

    Trigger rotateRobot180 = new JoystickButton(chassis, XboxController.Axis.kRightTrigger.value).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(180);
    }));

    Trigger rotateRobotTo0 = new JoystickButton(chassis, XboxController.Axis.kLeftTrigger.value).onTrue(new InstantCommand(() -> {
      Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnToOriginOriented(0);
      ;
    }));
    
    ArrayList<SwervePoint> posList = new ArrayList<SwervePoint>();

    Trigger FollowRoute = new JoystickButton(chassis, XboxController.Button.kStart.value).onTrue(
      new SequentialCommandGroup(new FollowRoute(posList),  teleop));

    Trigger savePoint = new JoystickButton(chassis, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> {
      posList.add(new SwervePoint(Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getX(),
                                  Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getY(),
                                  Swerve.getInstance(Consts.ChassisValues.USES_ABS_ENCODER).getGyro().getAngle()));
    }));

    Trigger removePoints = new JoystickButton(chassis, XboxController.Button.kBack.value).onTrue(new InstantCommand(()->{
      posList.clear();
    }));

    // Trigger turnToZero = new JoystickButton(controller, 1).whileTrue(new TurnToPoint(0, 0)); 
    // Trigger turnToPoint = new JoystickButton(controller, 4).whileTrue(new TurnToPoint(16, 5.6));
  
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
