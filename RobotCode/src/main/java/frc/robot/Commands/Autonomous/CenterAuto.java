package frc.robot.Commands.Autonomous;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Commands.Intake.IntakeWithPID;
import frc.robot.Commands.Intake.IntakeWithoutPID;
import frc.robot.Commands.Shooter.ShootToSpeaker;
import frc.robot.Commands.Shooter.TurnShooterToIntake;
import frc.robot.Commands.Shooter.TurnShooterToSpeaker;
import frc.robot.Commands.Swerve.DriveToPoint;
import frc.robot.Commands.Swerve.FollowRoute;
import frc.robot.Commands.Swerve.TurnToSpeaker;
import frc.robot.Commands.Swerve.TurnToSpeakerAuto;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Consts.ShooterValues;
import frc.robot.Utils.SwervePoint;
import frc.robot.Utils.Vector2d;

public class CenterAuto extends Command implements Consts{

    @Override
    public void initialize() {
        
        
        ChassisValues.AUTO_DRIVE_SPEED = 1.5;
        boolean blueAlliance = (Robot.getAlliance() == Alliance.Blue) ? true : false;
        
        new SequentialCommandGroup(
                                           
            //first note
            new TurnShooterToSpeaker().withTimeout(0.2)
            ,new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);})
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();})
            ,new WaitCommand(1)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})
            ,new WaitCommand(0.5)

            //second note   
            ,new TurnShooterToIntake()
            ,new DriveToPoint(new SwervePoint((blueAlliance) ? 1.4: PhysicalConsts.FIELD_WIDTH - 1.4, 5.4, (blueAlliance) ? 90 : 270)).withTimeout(1)
            ,new ParallelCommandGroup(
                new IntakeWithoutPID(IntakeValues.INTAKE_SPEED)
                ,new DriveToPoint(new SwervePoint((blueAlliance) ? 2.5 : PhysicalConsts.FIELD_WIDTH - 2.5, 5.4 , (blueAlliance) ? 90.5 : 270))
            ).until(new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return Shooter.getInstance().isNoteIn();
                }
            }).withTimeout(2)
            ,new InstantCommand(() -> {Shooter.getInstance().pullNoteWithoutPID(0);})

            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, 1),false);}).withTimeout(0.5)
            ,new TurnToSpeakerAuto()
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();})
            ,new TurnShooterToSpeaker().withTimeout(0.2)
            ,new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);})
            ,new WaitCommand(1.5)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})
            ,new WaitCommand(0.5)

            //third note 
            ,new TurnShooterToIntake()
            ,new DriveToPoint(new SwervePoint((blueAlliance) ? 2.5 : PhysicalConsts.FIELD_WIDTH - 2.5, (blueAlliance) ? 6.3 : 6.3, (blueAlliance) ? 0 : 0))
            ,new ParallelCommandGroup(
                new IntakeWithoutPID(IntakeValues.INTAKE_SPEED)
                ,new WaitCommand(0.2).andThen(new DriveToPoint(new SwervePoint((blueAlliance) ? 2.5 : PhysicalConsts.FIELD_WIDTH - 2.5, 6.65, (blueAlliance) ? 0 : 0)))
            ).until(new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return Shooter.getInstance().isNoteIn();
                }
            }).withTimeout(2)
            ,new InstantCommand(() -> {Shooter.getInstance().pullNoteWithoutPID(0);})
            ,new TurnToSpeakerAuto()
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();})
            ,new TurnShooterToSpeaker().withTimeout(0.2)
            ,new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);})
            ,new WaitCommand(1.5)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})
            ,new WaitCommand(0.5)

            ,new TurnShooterToIntake()
            ,new InstantCommand(() -> {Shooter.getInstance().stopRollers();})

            //fourth note
            /* 
            ,new TurnShooterToIntake()
            ,new DriveToPoint(new SwervePoint((blueAlliance) ? 2.9 - INTAKE_LOCALIZATION_OFFSET : 0, (blueAlliance) ? 4.7 : 0, (blueAlliance) ? 180 : 0))
            ,new ParallelCommandGroup(
                new IntakeWithoutPID(IntakeValues.INTAKE_SPEED)
                ,new WaitCommand(0.2).andThen(new DriveToPoint(new SwervePoint((blueAlliance) ? 2.9 : 0, 4.35, (blueAlliance) ? 0 : 180)))
            ).until(new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return Shooter.getInstance().isNoteIn();
                }
            }).withTimeout(2)
            ,new TurnToSpeakerAuto()
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();})
            ,new TurnShooterToSpeaker().withTimeout(0.2)
            ,new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);})
            ,new WaitCommand(1.5)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})
            */
        ).schedule();

    }
    
}
