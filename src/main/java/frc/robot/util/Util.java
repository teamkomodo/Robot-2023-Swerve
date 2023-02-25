package frc.robot.util;

public class Util {

    public static double joystickCurve(double input) {
        //Curve is f(x) = lx^3 + (1 - l)x
        //l = 0 => linear
        //l = 1 => cubic
        double sensitivityConstant = 1;
        return sensitivityConstant * input * input * input + (1 - sensitivityConstant) * input;   
    }
}
