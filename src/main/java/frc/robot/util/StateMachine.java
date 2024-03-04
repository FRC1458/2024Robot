package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

public class StateMachine<S> {

    public interface BasicState {
        void run();
    }

    private final Map<S, Function<Long, S>> stateMap = new HashMap<>();

    private final S defaultState;
    private S currentState;
    private long timestamp;

    public StateMachine (S defaultState) {
        this.defaultState = defaultState;
        currentState = defaultState;
    }

    public void addState(S state, Function<Long, S> function) {
        stateMap.put(state, function);
    }

    public void addTimerState(S state, long time, S nextState, Supplier<S> function) {
        stateMap.put(state, (ts) -> {
            S result = function.get();
            if (ts > time) {
                return nextState;
            }
            return result;
        });
    }

    public void addBoolState(S state, S nextState, Supplier<Boolean> function) {
        stateMap.put(state, (ts) -> function.get() ? nextState : state);
    }

    public void addTimerState(S state, long time, S nextState, BasicState function) {
        addTimerState(state, time, nextState, () -> {
            function.run();
            return null;
        });
    }

    public void addOffState(S state, BasicState function) {
        addState(state, (ts) -> {
            function.run();
            return null;
        });
    }

    public void reset() {
        timestamp = System.currentTimeMillis();
        currentState = defaultState;
    }

    public void run() {
        if (!stateMap.containsKey(currentState)) {
            System.out.println("current state error");
            return;
        }

        S result = stateMap.get(currentState).apply(System.currentTimeMillis() - timestamp);
        if (result != null && result != currentState) {
            currentState = result;
            timestamp = System.currentTimeMillis();
        }
    }

}