// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.DistanceCalculator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Takes a series of firing solution and performs linear interpolation between
 * them to provide a Firing Solution at any distance
 */
public class DistanceCalculator {
    private VisionMeasurement m_defaultVisionMeasurement = new VisionMeasurement(0.0, 10.0);
    private boolean m_sorted = false;

    private List<VisionMeasurement> m_data = new ArrayList<VisionMeasurement>();

    public void addSolution(VisionMeasurement f) {
        m_data.add(f);
        m_sorted = false;
    }

    private void sort() {
        Collections.sort(m_data, new VisionMeasurementSortByReading());
    }

    public void setDefaultFiringSolution(VisionMeasurement f) {
        m_defaultVisionMeasurement = f;
    }

    /**
     * Returns a FiringSolution with the pitch and distance interpolated. If a
     * FiringSolution can't be found return the default firing solution.
     * 
     * @param measurement
     * @return
     */
    public VisionMeasurement compute(double measurement) {
        if (!m_sorted) {
            this.sort();
            m_sorted = true;
        }

        // make sure we have at least 2 data points
        if (m_data.size() < 2) {
            return m_defaultVisionMeasurement;
        }

        // make sure that the distance is in range
        // and handle short special cases
        if (measurement < m_data.get(0).m_measurement) {
            return m_defaultVisionMeasurement;
        } else if (m_data.get(0).m_measurement == measurement) {
            return m_data.get(0);
        }

        // find the 2 closest firing solutions that the distance is between
        // once found interpolate between them
        VisionMeasurement low, high;
        for (int c = 1; c < m_data.size(); c++) {
            low = m_data.get(c - 1);
            high = m_data.get(c);

            if (low.m_measurement < measurement && high.m_measurement >= measurement) {
                return interpolate(measurement, low, high);
            }
        }

        return m_defaultVisionMeasurement;
    }

    private VisionMeasurement interpolate(double measurement, VisionMeasurement low, VisionMeasurement high) {
        double deltaDistance = high.m_distance - low.m_distance;
        double deltaMeasurement = high.m_measurement - low.m_measurement;

        double slopeDistance = deltaDistance / deltaMeasurement;

        VisionMeasurement answer = new VisionMeasurement();
        answer.m_measurement = measurement;
        answer.m_distance = low.m_distance + ((measurement - low.m_measurement) * slopeDistance);

        return answer;
    }
}
