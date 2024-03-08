package frc.robot.Commands.Autonomous;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Shooter.TurnShooterToSpeaker;
import frc.robot.Commands.Swerve.TurnToSpeakerAuto;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;
import frc.robot.Utils.Consts.ShooterValues;

public class NotCenterAuto extends Command implements Consts{
    
    @Override
    public void initialize() {
        new SequentialCommandGroup(
                new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, 1), false);})
                ,new WaitCommand(1)
                ,new InstantCommand(() -> {Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, 0), false);})                                                  
                ,new ParallelCommandGroup(new TurnToSpeakerAuto())
                ,new ParallelCommandGroup(
                                          new TurnShooterToSpeaker(), new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);})).until(new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                      return Shooter.getInstance().isReadyToShoot();
                    }
                 }).withTimeout(2.5)
                ,new WaitCommand(0.5)
                ,new InstantCommand(() ->{ Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})
                ,new WaitCommand(0.5)
                ,new InstantCommand(() ->{ Shooter.getInstance().pushNoteToRollers(0);})
                ,new InstantCommand(() -> {Shooter.getInstance().stopRollers(); Intake.getInstance().stopMotor();})
        ).schedule();
    }



}
