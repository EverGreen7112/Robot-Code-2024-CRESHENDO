package frc.robot.Utils;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.Utils.Consts.ShooterValues;

public class Funcs {

  public static Vector2d getSpeaker2d(){
    Vector3d speaker3d = new Vector3d();
        if (Robot.getAlliance() == Alliance.Blue) {
          speaker3d = ShooterValues.BLUE_SPAKER_POS;
        } 
        else if (Robot.getAlliance() == Alliance.Red) {
          speaker3d = ShooterValues.RED_SPAKER_POS;
        }
        return new Vector2d(speaker3d.m_x, speaker3d.m_z);
  }


    public static double closestAngle(double a, double b) {
        // get direction
        double dir = modulo(b, 360.0) - modulo(a, 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }

    /**
     * 
     * real modulo operation instead of %
     */
    public static double modulo(double a, double b) {
        return ((a % b) + b) % b;
    }

    /**
     * 
     * @param num - input number
     * @param amount - number of digits after decimal point
     * @return - rounded value 
     */
    public static double roundAfterDecimalPoint(double num, int amount) {
        num *= Math.pow(10, amount);
        num = (int) num;
        num /= Math.pow(10, amount);
        return num;
    }

    public static double convertRotationsToDegrees(double rotations){
        //convert rotations to degrees
        rotations *= 360;
       
        //convert from -180 - 180 to 0 - 360 
        if(rotations < 0){
            rotations += 360;
        }
        return rotations;
    }
}
