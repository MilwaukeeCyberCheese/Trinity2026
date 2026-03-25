package edu.msoe.cybercheese.trinity.auto;

public final class AutoRoutes {
    public static final String AIM_AND_SHOOT = "AimAndShoot";

    public static final String LEFT_START = "Left Start";
    public static final String CENTER_START = "Center Start";
    public static final String RIGHT_START = "Right Start";

    public static final String CENTER_SHOOT_CENTER = "CenterShootCenter";
    public static final String SWEEP_AROUND_LEFT = "SweepAroundLeft";
    public static final String SWEEP_AROUND_RIGHT = "SweepAroundRight";
    public static final String LEFT_IN_SHOOT = "LeftInShoot";
    public static final String RIGHT_IN_SHOOT = "RightInShoot";
    public static final String LEFT_SHOOT_CENTER = "LeftShootCenter";
    public static final String RIGHT_SHOOT_CENTER = "RightShootCenter";
    public static final String SIMPLE_LEFT_SWEEP = "SimpleLeftSweep";
    public static final String LEFT_SHOOT_LEFT = "LeftShootLeft";
    public static final String RIGHT_SHOOT_RIGHT = "RightShootRight";
    public static final String SIMPLE_RIGHT_SWEEP = "SimpleRightSweep";

    public static final AutoRouteGraph GRAPH = AutoRouteGraph.builder()
            .addNode(LEFT_START)
            .addNode(CENTER_START)
            .addNode(RIGHT_START)
            .addNode(CENTER_SHOOT_CENTER, CENTER_SHOOT_CENTER, AIM_AND_SHOOT)
            .addNode(SWEEP_AROUND_LEFT, SWEEP_AROUND_LEFT)
            .addNode(SWEEP_AROUND_RIGHT, SWEEP_AROUND_RIGHT)
            .addNode(LEFT_IN_SHOOT, LEFT_IN_SHOOT, AIM_AND_SHOOT)
            .addNode(RIGHT_IN_SHOOT, RIGHT_IN_SHOOT, AIM_AND_SHOOT)
            .addNode(LEFT_SHOOT_CENTER, LEFT_SHOOT_CENTER, AIM_AND_SHOOT)
            .addNode(RIGHT_SHOOT_CENTER, RIGHT_SHOOT_CENTER, AIM_AND_SHOOT)
            .addNode(SIMPLE_LEFT_SWEEP, SIMPLE_LEFT_SWEEP)
            .addNode(LEFT_SHOOT_LEFT, LEFT_SHOOT_LEFT, AIM_AND_SHOOT)
            .addNode(RIGHT_SHOOT_RIGHT, RIGHT_SHOOT_RIGHT, AIM_AND_SHOOT)
            .addNode(SIMPLE_RIGHT_SWEEP, SIMPLE_RIGHT_SWEEP)
            .addRoots(LEFT_START, CENTER_START, RIGHT_START)
            .connect(CENTER_START, CENTER_SHOOT_CENTER)
            .connect(CENTER_SHOOT_CENTER, SWEEP_AROUND_LEFT)
            .connect(CENTER_SHOOT_CENTER, SWEEP_AROUND_RIGHT)
            .connect(SWEEP_AROUND_RIGHT, LEFT_IN_SHOOT)
            .connect(SWEEP_AROUND_LEFT, RIGHT_IN_SHOOT)
            .connect(LEFT_START, LEFT_SHOOT_CENTER)
            .connect(LEFT_SHOOT_CENTER, SWEEP_AROUND_LEFT)
            .connect(LEFT_SHOOT_CENTER, SWEEP_AROUND_RIGHT)
            .connect(RIGHT_START, RIGHT_SHOOT_CENTER)
            .connect(RIGHT_SHOOT_CENTER, SWEEP_AROUND_LEFT)
            .connect(RIGHT_SHOOT_CENTER, SWEEP_AROUND_RIGHT)
            .connect(LEFT_START, SIMPLE_LEFT_SWEEP)
            .connect(SIMPLE_LEFT_SWEEP, RIGHT_IN_SHOOT)
            .connect(LEFT_START, LEFT_SHOOT_LEFT)
            .connect(RIGHT_START, RIGHT_SHOOT_RIGHT)
            .connect(RIGHT_START, SIMPLE_RIGHT_SWEEP)
            .build();

    private AutoRoutes() {}
}
