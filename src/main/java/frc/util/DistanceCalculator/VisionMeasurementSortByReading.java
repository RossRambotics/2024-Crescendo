// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.DistanceCalculator;

import java.util.Comparator;

/** Add your docs here. */
public class VisionMeasurementSortByReading implements Comparator<VisionMeasurement> {

    @Override
    public int compare(VisionMeasurement arg0, VisionMeasurement arg1) {
        VisionMeasurement one = (VisionMeasurement) arg0;
        VisionMeasurement two = (VisionMeasurement) arg1;

        if (one.m_measurement < two.m_measurement) {
            return -1;
        } else if (one.m_measurement > two.m_measurement) {
            return 1;
        }
        return 0;
    }
}
