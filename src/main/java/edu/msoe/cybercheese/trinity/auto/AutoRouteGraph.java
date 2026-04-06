package edu.msoe.cybercheese.trinity.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;

public final class AutoRouteGraph {

    public static final String STOP_ID = "__STOP__";

    private final Map<String, Node> nodes;
    private final List<String> rootNodeIds;

    private AutoRouteGraph(final Map<String, Node> nodes, final List<String> rootNodeIds) {
        this.nodes = Map.copyOf(nodes);
        this.rootNodeIds = List.copyOf(rootNodeIds);
    }

    public static Builder builder() {
        return new Builder();
    }

    public List<Node> roots() {
        return this.rootNodeIds.stream().map(this::node).toList();
    }

    public List<List<String>> stopTerminatedPaths() {
        final var paths = new ArrayList<List<String>>();
        for (final var rootNodeId : this.rootNodeIds) {
            final var currentPath = new ArrayList<String>();
            currentPath.add(rootNodeId);
            this.collectStopTerminatedPaths(rootNodeId, currentPath, paths);
        }
        return List.copyOf(paths);
    }

    public Node node(final String id) {
        final var node = this.nodes.get(id);
        if (node == null) {
            throw new IllegalArgumentException("Unknown auto route node: " + id);
        }
        return node;
    }

    public List<Choice> choicesFrom(final String nodeId) {
        final var node = this.node(nodeId);
        final var choices = new ArrayList<Choice>();
        if (node.canStop()) {
            choices.add(Choice.stop(node.id()));
        }
        choices.addAll(node.transitions());
        return List.copyOf(choices);
    }

    public List<Node> pathOf(final List<String> nodeIds) {
        if (nodeIds.isEmpty()) {
            return List.of();
        }

        final var path = new ArrayList<Node>();
        final var rootIds = Set.copyOf(this.rootNodeIds);

        @Nullable String previousId = null;
        for (final var nodeId : nodeIds) {
            final var node = this.node(nodeId);
            if (previousId == null) {
                if (!rootIds.contains(nodeId)) {
                    throw new IllegalArgumentException("Path must start at a root node, got: " + nodeId);
                }
            } else if (!this.hasTransition(previousId, nodeId)) {
                throw new IllegalArgumentException("No transition from " + previousId + " to " + nodeId);
            }

            path.add(node);
            previousId = nodeId;
        }

        return List.copyOf(path);
    }

    public Command commandForPath(
            final List<String> nodeIds,
            final Function<String, Command> routeCommandFactory,
            final Function<String, Command> transitionCommandFactory,
            final Function<String, Command> postRouteCommandFactory) {
        final var path = this.pathOf(nodeIds);
        if (path.isEmpty()) {
            return Commands.none();
        }

        final var commands = new ArrayList<Command>();

        for (int i = 0; i < path.size(); i++) {
            final var node = path.get(i);
            if (node.routeName() != null) {
                commands.add(routeCommandFactory.apply(node.routeName()));
                if (node.postRouteActionId() != null) {
                    commands.add(postRouteCommandFactory.apply(node.postRouteActionId()));
                }
            }

            if (i + 1 >= path.size()) {
                continue;
            }

            final var transition =
                    this.transition(path.get(i).id(), path.get(i + 1).id());
            if (transition.transitionActionId() != null) {
                commands.add(transitionCommandFactory.apply(transition.transitionActionId()));
            }
        }

        return commands.isEmpty() ? Commands.none() : Commands.sequence(commands.toArray(Command[]::new));
    }

    public boolean hasTransition(final String fromId, final String toId) {
        return this.node(fromId).transitions().stream()
                .anyMatch(choice -> choice.nextNodeId().equals(toId));
    }

    public Choice transition(final String fromId, final String toId) {
        return this.node(fromId).transitions().stream()
                .filter(choice -> choice.nextNodeId().equals(toId))
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException("No transition from " + fromId + " to " + toId));
    }

    public record Node(
            String id,
            @Nullable String routeName,
            @Nullable String postRouteActionId,
            List<Choice> transitions,
            boolean canStop) {
        public Node {
            Objects.requireNonNull(id);
            transitions = List.copyOf(transitions);
        }

        public boolean isRoute() {
            return this.routeName != null;
        }
    }

    public record Choice(String label, String nextNodeId, @Nullable String transitionActionId, boolean stopChoice) {
        public Choice {
            Objects.requireNonNull(label);
            Objects.requireNonNull(nextNodeId);
        }

        public static Choice stop(final String fromNodeId) {
            return new Choice("Stop", STOP_ID + "@" + fromNodeId, null, true);
        }
    }

    public static final class Builder {
        private final Map<String, NodeBuilder> nodes = new LinkedHashMap<>();
        private final List<String> rootNodeIds = new ArrayList<>();

        public Builder addNode(final String id) {
            return this.addNode(id, null, null);
        }

        public Builder addNode(final String id, final @Nullable String routeName) {
            return this.addNode(id, routeName, null);
        }

        public Builder addNode(
                final String id, final @Nullable String routeName, final @Nullable String postRouteActionId) {
            this.nodes.compute(id, (ignored, existing) -> {
                if (existing != null) {
                    throw new IllegalArgumentException("Duplicate auto route node: " + id);
                }
                return new NodeBuilder(id, routeName, postRouteActionId);
            });
            return this;
        }

        public Builder addRoot(final String id) {
            if (!this.nodes.containsKey(id)) {
                throw new IllegalArgumentException("Root node must be declared first: " + id);
            }
            this.rootNodeIds.add(id);
            return this;
        }

        public Builder addRoots(final String... ids) {
            for (final var id : ids) {
                this.addRoot(id);
            }
            return this;
        }

        public Builder connect(final String fromId, final String toId) {
            return this.connect(fromId, toId, null, null);
        }

        public Builder connect(final String fromId, final String toId, final @Nullable String transitionActionId) {
            return this.connect(fromId, toId, null, transitionActionId);
        }

        public Builder connect(
                final String fromId,
                final String toId,
                final @Nullable String label,
                final @Nullable String transitionActionId) {
            final var from = this.nodeBuilder(fromId);
            this.nodeBuilder(toId);

            final var choiceLabel = label != null ? label : toId;
            from.transitions.add(new Choice(choiceLabel, toId, transitionActionId, false));
            return this;
        }

        public Builder canStop(final String nodeId, final boolean canStop) {
            this.nodeBuilder(nodeId).canStop = canStop;
            return this;
        }

        public AutoRouteGraph build() {
            if (this.rootNodeIds.isEmpty()) {
                throw new IllegalStateException("Auto route graph must define at least one root node");
            }

            final var builtNodes = new LinkedHashMap<String, Node>();
            for (final var entry : this.nodes.entrySet()) {
                final var node = entry.getValue();
                builtNodes.put(
                        node.id,
                        new Node(node.id, node.routeName, node.postRouteActionId, node.transitions, node.canStop));
            }

            return new AutoRouteGraph(builtNodes, this.rootNodeIds);
        }

        private NodeBuilder nodeBuilder(final String nodeId) {
            final var builder = this.nodes.get(nodeId);
            if (builder == null) {
                throw new IllegalArgumentException("Unknown auto route node: " + nodeId);
            }
            return builder;
        }
    }

    private static final class NodeBuilder {
        private final String id;
        private final @Nullable String routeName;
        private final @Nullable String postRouteActionId;
        private final List<Choice> transitions = new ArrayList<>();
        private boolean canStop = true;

        private NodeBuilder(
                final String id, final @Nullable String routeName, final @Nullable String postRouteActionId) {
            this.id = id;
            this.routeName = routeName;
            this.postRouteActionId = postRouteActionId;
        }
    }

    private void collectStopTerminatedPaths(
            final String nodeId, final List<String> currentPath, final List<List<String>> paths) {
        final var node = this.node(nodeId);
        if (node.canStop()) {
            paths.add(List.copyOf(currentPath));
        }
        for (final var transition : node.transitions()) {
            currentPath.add(transition.nextNodeId());
            this.collectStopTerminatedPaths(transition.nextNodeId(), currentPath, paths);
            currentPath.remove(currentPath.size() - 1);
        }
    }
}
