package frc.robot.Commands.Autonomous;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Intake.IntakeWithoutPID;
import frc.robot.Commands.Shooter.TurnShooterToAngle;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;
import frc.robot.Utils.Consts.ShooterValues;

public class TwoNoteAndPickupAmpSideAuto extends Command implements Consts{
    //fll auto
    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new ParallelCommandGroup(new InstantCommand(() -> {Shooter.getInstance().turnToAngle(114);}),
                                     new InstantCommand(()->{Shooter.getInstance().setShootSpeed(7000, ShooterValues.SPEAKER_SHOOT_SPEED * 1.1 / 3);}))
            ,new WaitCommand(2)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED); }) 
            ,new WaitCommand(0.5)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(0); Shooter.getInstance().setShootSpeed(0);})
            ,new ParallelCommandGroup(
                new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, -1.4), true);}),
                new IntakeWithoutPID(IntakeValues.INTAKE_SPEED)
            ).until(new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return Shooter.getInstance().isNoteIn();
                }
            }).withTimeout(2)       
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, 0), true);
                                       new IntakeWithoutPID(0); })
            ,new WaitCommand(0.5)
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, 1.4), true);})
            ,new WaitCommand(2)
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, 0), true);})
            ,new ParallelCommandGroup(new InstantCommand(() -> {Shooter.getInstance().turnToAngle(114);}),
                                     new InstantCommand(()->{Shooter.getInstance().setShootSpeed(7000, ShooterValues.SPEAKER_SHOOT_SPEED * 1.1 / 3);}))
            ,new WaitCommand(2)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);}) 
            ,new WaitCommand(0.5)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(0);
                                       Shooter.getInstance().setShootSpeed(0); 
                                       Shooter.getInstance().turnToAngle(ShooterValues.AIM_MOTOR_MIN_ANGLE);})
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, -1.65), true);})
            ,new WaitCommand(1)
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnToOriginOriented(270);})
            ,new ParallelCommandGroup(    
                new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(1.45, 0), true);})
                ,new IntakeWithoutPID(IntakeValues.INTAKE_SPEED)
            ).withTimeout(1)
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, 0), true);})          
        ).schedule();
    }



}
