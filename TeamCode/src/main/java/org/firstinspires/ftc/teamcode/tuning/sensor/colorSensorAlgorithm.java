package org.firstinspires.ftc.teamcode.tuning.sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class colorSensorAlgorithm {

    public ColorSensor internalColorSensor;
    public DistanceSensor internalDistanceSensor;
    // Base Color Threshold
    public final double RED_THRESHOLD_LEFT = 700, RED_THRESHOLD_RIGHT = 700;
    public final double BLUE_THRESHOLD_LEFT = 700, BLUE_THRESHOLD_RIGHT = 700;
    public final double RED_THRESHOLD = 550;
    public final double BLUE_THRESHOLD = 550;
    // Complex Color Thresholds (not red, green, or blue)
    public final double YELLOW_RED_VALUE = 0.419;
    public final double YELLOW_GREEN_VALUE = 0.3675;
    public final double YELLOW_BLUE_VALUE = 0.2116;
    public final double[] YELLOW_CONSTANTS = {YELLOW_RED_VALUE, YELLOW_GREEN_VALUE, YELLOW_BLUE_VALUE};
    public final double YELLOW_THRESHOLD = 0.12;

    public final double WHITE_RED_VALUE = 0.30125;
    public final double WHITE_GREEN_VALUE = 0.37125;
    public final double WHITE_BLUE_VALUE = 0.325;
    public final double[] WHITE_CONSTANTS = {WHITE_RED_VALUE, WHITE_GREEN_VALUE, WHITE_BLUE_VALUE};
    public final double WHITE_THRESHOLD = 0.01;
    public final double WHITE_TOTAL_COUNT = 800;

    // check for black with the alpha value
    public final double BLACK_ALPHA_VALUE = 325; //Test value

    public colorSensorAlgorithm(HardwareMap hardwareMap, String name){
        this.internalColorSensor = hardwareMap.get(ColorSensor.class, name);
        this.internalDistanceSensor = hardwareMap.get(DistanceSensor.class, name);
    }

    public double distance(){
        return internalDistanceSensor.getDistance(DistanceUnit.CM);
    }


    // utility method for easily getting the color value.
    public int red(){ return internalColorSensor.red(); }
    public int green(){ return internalColorSensor.green(); }
    public int blue(){ return internalColorSensor.blue(); }

    public double total(){ return red() + green() + blue(); }

    // convert to array
    public double[] rgb(){
        double[] arr = new double[3];
        arr[0] = red();
        arr[1] = green();
        arr[2] = blue();

        return arr;
    }

    // Scale the rgb values (0 to 1)
    public double[] normalizedRGB(){
        double[] arr = new double[3];
        double[] originalArr = rgb();

        double total = 0;
        for(double i : originalArr)
            total+= i;

        for(int i = 0; i < 3; i++){
            arr[i] = originalArr[i] / total;
        }

        return arr;
    }

    public double arrayError(double[] arr1, double[] arr2){
        double total = 0;

        for(int i = 0; i < arr1.length; i++){
            total += Math.pow(arr1[i] - arr2[i], 2);
        }

        return Math.sqrt(total);
    }

    public boolean isBlack(){
        return (internalColorSensor.alpha() < BLACK_ALPHA_VALUE) ? true : false;
    }

    public int alphaValue(){
        return internalColorSensor.alpha();
    }

    public boolean isRed(){
        return internalColorSensor.red() > RED_THRESHOLD;
    }

    public boolean isBlue(){
        return internalColorSensor.blue() > BLUE_THRESHOLD;
    }

    public double yellowError(){ return arrayError(normalizedRGB(), YELLOW_CONSTANTS); }
    public double whiteError(){ return arrayError(normalizedRGB(), WHITE_CONSTANTS); }

    public boolean isYellow(){
        return yellowError() < YELLOW_THRESHOLD || (internalDistanceSensor.getDistance(DistanceUnit.CM)>6 && whiteError() > 0.02);
    }

    public boolean isWhite(){
        return whiteError() < WHITE_THRESHOLD && total() > WHITE_TOTAL_COUNT;
    }

    public String normalizedValues() {
        double red = internalColorSensor.red();
        double green = internalColorSensor.green();
        double blue = internalColorSensor.blue();

        double total = red + green + blue;
        return String.format("RGB: %.2f %.2f %.2f", red / total, green / total, blue / total);
    }

    // turn on the lights
    public void enableLED(boolean LEDMode){
        internalColorSensor.enableLed(LEDMode);
    }

    public boolean withinColorRange(){
        return isYellow() || isWhite();
    }
}