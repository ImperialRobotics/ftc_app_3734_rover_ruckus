package org.firstinspires.ftc.teamcode.refactor.utils;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

public class PIDController {
    public double lastError, totalError;
    public double targetPosition = 0;
    public double minInput, maxInput, minOutput, maxOutput;
    public boolean inputBounded, outputBounded;
    public PIDCoefficients pidCoefficients;


    public PIDController(double minOuput, double maxOutput) {
        inputBounded = false;
        outputBounded = true;
        this.minOutput = minOuput;
        this.maxOutput = maxOutput;
    }

    public PIDController(double minInput, double maxInput, double minOutput, double maxOutput) {
        this.minInput = minInput;
        this.maxInput = maxInput;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        inputBounded = true;
        outputBounded = true;
    }

    public double getError(double position) {
        if(inputBounded)
            position = Range.clip(position, minInput, maxInput);
        double error = 0.0;

        if(outputBounded)
            error = Range.clip(targetPosition - position, minInput, maxInput);
        else
            error = targetPosition - position;
        while (error > 180)
            error -= 360;
        while (error <= -180)
            error += 360;
        return error;
    }

    public double update(double position) {
        double error = getError(position);

        double proportion = error * pidCoefficients.p;
        double integral = totalError * pidCoefficients.i;
        double derivative = (error - lastError) * pidCoefficients.d;

        lastError = error;
        totalError += error;

        double output = proportion + integral + derivative;

        if (outputBounded)
            return Range.clip(output, minOutput, maxOutput);
        else
            return output;
    }


    public void reset() {
        totalError = 0.0;
        lastError = 0.0;
    }
}
