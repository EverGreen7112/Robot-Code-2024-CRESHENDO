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
        
        boolean blueAlliance = (Robot.getAlliance() == Alliance.Blue) ? true : false;
        
        new SequentialCommandGroup(
                                           
            //first note
            new TurnShooterToSpeaker().withTimeout(0.2)
            ,new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);})
            ,new TurnToSpeakerAuto()
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();;})
            ,new WaitCommand(1)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})
            
            //second note
            ,new DriveToPoint(new SwervePoint((blueAlliance) ? 1.65 : 0, 5.5, (blueAlliance) ? 93 : 270))
            ,new WaitCommand(0.2)
            ,new ParallelCommandGroup(
                new IntakeWithoutPID(IntakeValues.INTAKE_SPEED)
                ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, -0.5),false);})
            ).until(new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return Shooter.getInstance().isNoteIn();
                }
            }).withTimeout(2)
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, 0),false);})
            ,new TurnToSpeakerAuto()
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();;})
            ,new TurnShooterToSpeaker().withTimeout(0.2)
            ,new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);})
            ,new WaitCommand(1.5)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})

        ).schedule();

    }
    
}
