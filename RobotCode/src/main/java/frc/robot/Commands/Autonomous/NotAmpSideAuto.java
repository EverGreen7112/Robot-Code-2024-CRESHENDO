package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        new SequentialCommandGroup(
            //first note
            new DriveToPoint(new SwervePoint(1.13, 4.5, -46))
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
