package frc.robot.Commands.Autonomous;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Commands.Intake.IntakeWithoutPID;
import frc.robot.Commands.Shooter.ShootToSpeaker;
import frc.robot.Commands.Shooter.TurnShooterToSpeaker;
import frc.robot.Commands.Swerve.FollowRoute;
import frc.robot.Commands.Swerve.TurnToSpeaker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Funcs;
import frc.robot.Utils.SwervePoint;
import frc.robot.Utils.Vector2d;

public class ThreeNoteAuto extends Command implements Consts{ 
       
        public ThreeNoteAuto(){ 
            addRequirements(Swerve.getInstance(ChassisValues.USES_ABS_ENCODER)); 
        } 
     
        @Override 
        public void initialize() { 
            Vector2d speaker = Funcs.getSpeaker2d();
             
            Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).setOdometryVals(speaker.x + ((Robot.getAlliance() == Alliance.Blue) ? 1: -1), speaker.y,  
            Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getFieldOrientedAngle());
            

            ArrayList<SwervePoint> routeToFirstNote = new ArrayList<SwervePoint>();
            routeToFirstNote.add(new SwervePoint(speaker.x + ((Robot.getAlliance() == Alliance.Blue) ? 1: -1), speaker.y, 0));

            new SequentialCommandGroup(
                 //shoot first note to speaker 
                 new ParallelCommandGroup(new TurnToSpeaker(), new TurnShooterToSpeaker(), new ShootToSpeaker()).until(new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                      return Shooter.getInstance().isReadyToShoot();
                    }
                 }).withTimeout(2.5)
                ,new WaitCommand(0.5)
                ,new InstantCommand(() ->{ Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})
                ,new WaitCommand(0.5)
                ,new InstantCommand(() ->{ Shooter.getInstance().pushNoteToRollers(0);})
                //second note
                ,new ParallelCommandGroup(new FollowRoute(routeToFirstNote), new WaitCommand(1).andThen(new IntakeWithoutPID(IntakeValues.INTAKE_SPEED)))
                ,new ParallelCommandGroup(new TurnToSpeaker(), new TurnShooterToSpeaker(), new ShootToSpeaker()).until(new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                      return Shooter.getInstance().isReadyToShoot();
                    }
                 }).withTimeout(2.5)
                ,new WaitCommand(0.5)
                ,new InstantCommand(() ->{ Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);})
                ,new WaitCommand(0.5)
                ,new InstantCommand(() ->{ Shooter.getInstance().pushNoteToRollers(0);})
                //third note
                
            ).schedule();
        } 
        @Override 
        public void execute() { 
    
             
        } 
     
        @Override 
        public void end(boolean interrupted) { 
            Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop(); 
        } 
     
}
