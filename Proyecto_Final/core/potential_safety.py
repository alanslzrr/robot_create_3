from . import potential_config as config

def saturate_wheel_speeds(v_left, v_right):
    max_abs = max(abs(v_left), abs(v_right))
    if max_abs > config.V_MAX_CM_S:
        scale = config.V_MAX_CM_S / max_abs
        v_left *= scale
        v_right *= scale
    v_left = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_left))
    v_right = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_right))
    return v_left, v_right

def detect_obstacle(ir_sensors):
    if not ir_sensors or len(ir_sensors) < 7:
        return {'stop': False, 'slow': False, 'max_front': 0, 'speed_factor': 1.0}
    front_vals = [
        ir_sensors[config.IR_SIDE_LEFT],
        ir_sensors[config.IR_FRONT_LEFT],
        ir_sensors[config.IR_FRONT_CENTER],
        ir_sensors[config.IR_FRONT_RIGHT]
    ]
    max_front = max(front_vals)
    stop = max_front > config.IR_THRESHOLD_STOP
    slow = max_front > config.IR_THRESHOLD_SLOW
    if max_front > config.IR_THRESHOLD_STOP:
        speed_factor = 0.0
    elif max_front > config.IR_THRESHOLD_SLOW:
        speed_factor = max(0.3, 1.0 - (max_front - config.IR_THRESHOLD_SLOW) /
                           (config.IR_THRESHOLD_STOP - config.IR_THRESHOLD_SLOW))
    else:
        speed_factor = 1.0
    return {
        'stop': stop,
        'slow': slow,
        'max_front': max_front,
        'speed_factor': speed_factor
    }

def emergency_stop_needed(bumpers):
    return bumpers[0] or bumpers[1]





