import edu.wpi.first.wpilibj.DriverStation;

public class PhaseTimer {
    private String lastAnnouncement = "";

    public void periodic() {
        double matchTime = DriverStation.getMatchTime();

        // Match time is unavailable outside a match or before the clock starts.
        if (matchTime < 0) {
            lastAnnouncement = "";
            return;
        }

        String phase;
        double secondsRemaining;

        if (matchTime > 135) {
            phase = "AUTO";
            secondsRemaining = matchTime - 135;
        } else if (matchTime > 125) {
            phase = "TRANSITION";
            secondsRemaining = matchTime - 125;
        } else if (matchTime > 95) {
            phase = "PHASE 1";
            secondsRemaining = matchTime - 95;
        } else if (matchTime > 65) {
            phase = "PHASE 2";
            secondsRemaining = matchTime - 65;
        } else if (matchTime > 35) {
            phase = "PHASE 3";
            secondsRemaining = matchTime - 35;
        } else if (matchTime > 30) {
            phase = "PHASE 4";
            secondsRemaining = matchTime - 30;
        } else {
            phase = "ENDGAME";
            secondsRemaining = matchTime;
        }

        int countdown = (int) Math.ceil(secondsRemaining);
        String announcement = phase + ": " + countdown + " seconds";

        if (!announcement.equals(lastAnnouncement)) {
            System.out.println(announcement);
            lastAnnouncement = announcement;
        }
    }
}