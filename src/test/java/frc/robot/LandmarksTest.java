package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class LandmarksTest {

    @Test
    void fieldDimensionsArePositive() {
        assertTrue(
            Landmarks.fieldLength > 5.0,
            "Field length should be populated from the AprilTag layout"
        );
        assertTrue(
            Landmarks.fieldWidth > 2.0,
            "Field width should be populated from the AprilTag layout"
        );
    }
}
