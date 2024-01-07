// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Other;

/** Add your docs here. */
public class SampleSmoother {
    protected double[] samples;
    protected double accumulator = 0.0;
    protected int count = 0;
    protected double smooth = 0.0;
    protected int numSamples = 0;

    public SampleSmoother(int sampleSize) {
        numSamples = sampleSize;
        samples = new double[numSamples];
        for (int i = 0; i < numSamples; i++) {
            samples[i] = 0.0;
        }
    }

    public double addSample(double newSample) {
        accumulator -= samples[count];
        samples[count] = newSample;
        accumulator += samples[count];
        smooth = accumulator / numSamples;
        count += 1;
        count = (count >= numSamples) ? 0 : count;
        return smooth;
    }

    public double getAverage() {
        return smooth;
    }

    public void reset() {
        accumulator = 0.0;
        count = 0;
        smooth = 0.0;
        for (int i = 0; i < numSamples; i++) {
            samples[i] = 0.0;
        }
    }
}
