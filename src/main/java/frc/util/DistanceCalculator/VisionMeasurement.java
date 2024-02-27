// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.DistanceCalculator;

/** Stores an optimate distance, pitch and speed */
public class VisionMeasurement {
    public double m_distance = 0;
    public double m_measurement = 0;

    public VisionMeasurement() {

    }

    /**
     * Creates and inializes a FiringSolution
     * 
     * @param distance
     * @param measurement
     */
    public VisionMeasurement(double measurement, double distance) {
        m_distance = distance;
        m_measurement = measurement;
    }
}
