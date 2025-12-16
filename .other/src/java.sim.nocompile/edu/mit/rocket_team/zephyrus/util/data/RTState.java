package edu.mit.rocket_team.zephyrus.util.data;

public enum RTState {
    GROUND_TESTING(0),
    PRE_FLIGHT(1),
    FLIGHT(2),
    APOGEE(3),
    DISREEF(4),
    END(5);

    public int ID = 0;

    RTState(int ID) {
        this.ID = ID;
    }
}
