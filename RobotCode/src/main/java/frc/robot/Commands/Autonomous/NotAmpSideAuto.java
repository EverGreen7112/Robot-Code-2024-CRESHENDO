package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Commands.Intake.IntakeWithoutPID;
import frc.robot.Commands.Shooter.TurnShooterToSpeaker;
import frc.robot.Commands.Swerve.DriveToPoint;
import frc.robot.Commands.Swerve.TurnToSpeakerAuto;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.SwervePoint;
import frc.robot.Utils.Consts.ChassisValues;
import frc.robot.Utils.Consts.ShooterValues;

public class NotAmpSideAuto extends Command implements Consts{

    @Override
    public void initialize() {
        ChassisValues.AUTO_DRIVE_SPEED = 1;
        final double INTAKE_LOCALIZATION_OFFSET = 0.14;
        boolean blueAlliance = (Robot.getAlliance() == Alliance.Blue) ? true: false;

        new SequentialCommandGroup(
            //first note
            new DriveToPoint(new SwervePoint((blueAlliance) ? 1.13 : 0, (blueAlliance) ? 4.5 : 0,(blueAlliance) ? -46 : 0))
            ,new TurnToSpeakerAuto()
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();})
            ,new TurnShooterToSpeaker().withTimeout(0.2)
            ,new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);})
            ,new WaitCommand(1)
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();;})
            ,new WaitCommand(1)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})
            ,new WaitCommand(0.5)
            
            //pick up note from the middle
            ,new InstantCommand(() -> {Shooter.getInstance().turnToAngle(ShooterValues.AIM_MOTOR_MIN_ANGLE); 
                                       Shooter.getInstance().pushNoteToRollers(0);
                                       Shooter.getInstance().setShootSpeed(0);
                                    })
            ,new DriveToPoint(new SwervePoint((blueAlliance) ? 3.6: 0, (blueAlliance) ? 2.6 : 0, (blueAlliance) ? -310 : 0))
            ,new DriveToPoint(new SwervePoint((blueAlliance) ? 7.7 + INTAKE_LOCALIZATION_OFFSET: 0, (blueAlliance) ? 0.57 : 0, (blueAlliance) ? -270 : 0))
            ,new IntakeWithoutPID(IntakeValues.INTAKE_SPEED).withTimeout(0.2)
            ,new DriveToPoint(new SwervePoint((blueAlliance) ? 9.2 + INTAKE_LOCALIZATION_OFFSET: 0, (blueAlliance) ? 0.57 : 0, (blueAlliance) ? -270 : 0))
            ,new IntakeWithoutPID(0).withTimeout(0.1)

            //come back
            ,new DriveToPoint(new SwervePoint((blueAlliance) ? 3.1: 0, (blueAlliance) ? 1.5 : 0, (blueAlliance) ? -320 : 0))
            ,new DriveToPoint(new SwervePoint((blueAlliance) ? 1.13 : 0, (blueAlliance) ? 4.5 : 0,(blueAlliance) ? -46 : 0))
            ,new TurnToSpeakerAuto()
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();})
            ,new TurnShooterToSpeaker().withTimeout(0.2)
            ,new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);})
            ,new WaitCommand(1)
            ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();;})
            ,new WaitCommand(1)
            ,new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})
            ,new WaitCommand(0.5)

        ).schedule();
    }

    @Override
    public void execute() {
      
    }
    
}
