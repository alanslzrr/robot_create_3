"""Implementación completa adaptada desde PL4/src/potential_fields.py."""

import math
from . import potential_config as config

POTENTIAL_TYPES = ['linear', 'quadratic', 'conic', 'exponential']
_last_v_linear = 0.0

def reset_velocity_ramp():
    global _last_v_linear
    _last_v_linear = 0.0

def _wrap_pi(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad

def attractive_wheel_speeds(q, q_goal, k_lin=None, k_ang=None, potential_type='linear'):
    if k_lin is None:
        if potential_type == 'linear':
            k_lin = config.K_LINEAR
        elif potential_type == 'quadratic':
            k_lin = config.K_QUADRATIC
        elif potential_type == 'conic':
            k_lin = config.K_CONIC
        elif potential_type == 'exponential':
            k_lin = config.K_EXPONENTIAL
        else:
            k_lin = config.K_LINEAR

    if k_ang is None:
        k_ang = config.K_ANGULAR

    dx = q_goal[0] - q[0]
    dy = q_goal[1] - q[1]
    distance = math.hypot(dx, dy)
    theta_rad = math.radians(q[2])
    desired_angle = math.atan2(dy, dx)
    angle_error = _wrap_pi(desired_angle - theta_rad)

    if potential_type == 'linear':
        v_linear = k_lin * distance
    elif potential_type == 'quadratic':
        v_linear = k_lin * (distance ** 2) / 10.0
    elif potential_type == 'conic':
        d_sat = 100.0
        v_linear = k_lin * min(distance, d_sat) * 2.0
    elif potential_type == 'exponential':
        lambda_param = 50.0
        v_linear = k_lin * (1.0 - math.exp(-distance / lambda_param)) * 20.0
    else:
        v_linear = k_lin * distance

    if distance < config.TOL_DIST_CM:
        v_linear = 0.0
    else:
        v_linear = min(config.V_MAX_CM_S, v_linear)
        if distance < config.DECEL_ZONE_CM:
            decel_factor = distance / config.DECEL_ZONE_CM
            v_linear_decel = v_linear * decel_factor
            v_linear = max(v_linear_decel, config.V_APPROACH_MIN_CM_S)

        global _last_v_linear
        max_delta_v = config.ACCEL_RAMP_CM_S2 * config.CONTROL_DT
        if _last_v_linear < config.V_START_MIN_CM_S:
            v_linear = max(v_linear, config.V_START_MIN_CM_S)
        if v_linear > _last_v_linear:
            v_linear = min(v_linear, _last_v_linear + max_delta_v)
        _last_v_linear = v_linear

    angle_factor = math.cos(angle_error)
    if distance > 50.0:
        min_factor = 0.6
    elif distance > 20.0:
        min_factor = 0.4
    else:
        min_factor = 0.2
    if angle_factor < min_factor:
        angle_factor = min_factor
    v_linear *= angle_factor
    if distance > 30.0 and v_linear < config.V_START_MIN_CM_S:
        v_linear = config.V_START_MIN_CM_S

    omega = k_ang * angle_error
    omega_max_rad_s = config.W_MAX_CM_S / (config.WHEEL_BASE_CM / 2.0)
    omega = max(-omega_max_rad_s, min(omega_max_rad_s, omega))

    half_base = config.WHEEL_BASE_CM / 2.0
    if distance > config.TOL_DIST_CM and v_linear > 0:
        if distance > 30.0:
            min_wheel_speed = 4.0
        elif distance > 10.0:
            min_wheel_speed = 2.0
        else:
            min_wheel_speed = 0.0
        max_omega_for_arc = (v_linear - min_wheel_speed) / half_base
        if abs(omega) > max_omega_for_arc:
            omega = math.copysign(max_omega_for_arc, omega)

    v_left = v_linear - half_base * omega
    v_right = v_linear + half_base * omega

    if distance > config.TOL_DIST_CM * 2:
        if v_left < 0 or v_right < 0 and v_linear > 0:
            max_omega_positive = v_linear / half_base
            if omega > max_omega_positive:
                omega = max_omega_positive * 0.95
            elif omega < -max_omega_positive:
                omega = -max_omega_positive * 0.95
            v_left = v_linear - half_base * omega
            v_right = v_linear + half_base * omega

    v_left = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_left))
    v_right = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_right))

    info = {
        'potential_type': potential_type,
        'v_linear': v_linear,
        'omega': omega,
        'angle_error_deg': math.degrees(angle_error),
        'angle_factor': angle_factor
    }

    return v_left, v_right, distance, info

def normalize_ir_reading(ir_value, sensor_index):
    if sensor_index not in config.IR_SENSOR_SENSITIVITY_FACTORS:
        return ir_value
    factor = config.IR_SENSOR_SENSITIVITY_FACTORS[sensor_index]
    return ir_value / factor

def ir_value_to_distance(ir_value, sensor_index=None):
    if sensor_index is not None and sensor_index in config.IR_SENSOR_SENSITIVITY_FACTORS:
        ir_normalized = ir_value / config.IR_SENSOR_SENSITIVITY_FACTORS[sensor_index]
    else:
        ir_normalized = ir_value
    if ir_normalized < 25:
        return config.IR_MAX_DISTANCE_CM
    if ir_normalized >= 1000:
        distance = 5.0
    elif ir_normalized >= 60:
        distance = 5.0 * math.pow(1000.0 / ir_normalized, 0.65)
    else:
        distance = 5.0 * math.pow(1000.0 / ir_normalized, 0.70)
    distance = max(config.IR_MIN_DISTANCE_CM, min(distance, config.IR_MAX_DISTANCE_CM))
    if sensor_index is not None and sensor_index in config.IR_SENSOR_ANGLES:
        sensor_angle_deg = abs(config.IR_SENSOR_ANGLES[sensor_index])
        if sensor_angle_deg > 50:
            distance *= 1.15
        elif sensor_angle_deg > 30:
            distance *= 1.08
        elif sensor_angle_deg > 15:
            distance *= 1.03
    return distance

def detect_navigable_gaps(ir_sensors, q):
    if not ir_sensors or len(ir_sensors) < 7:
        return []
    gaps = []
    theta_robot_rad = math.radians(q[2])
    distances = [ir_value_to_distance(ir_sensors[i]) for i in range(7)]
    for i in range(7):
        if ir_sensors[i] < config.GAP_BLOCKED_THRESHOLD:
            continue
        for j in range(i + 1, min(i + 4, 7)):
            if ir_sensors[j] < config.GAP_BLOCKED_THRESHOLD:
                continue
            all_clear_between = True
            for k in range(i + 1, j):
                if ir_sensors[k] >= config.GAP_CLEAR_THRESHOLD:
                    all_clear_between = False
                    break
            if not all_clear_between:
                continue
            angle_i = config.IR_SENSOR_ANGLES.get(i, 0)
            angle_j = config.IR_SENSOR_ANGLES.get(j, 0)
            dist_i = distances[i]
            dist_j = distances[j]
            angle_i_rad = math.radians(angle_i)
            angle_j_rad = math.radians(angle_j)
            obs_i_local_x = dist_i * math.sin(angle_i_rad)
            obs_i_local_y = dist_i * math.cos(angle_i_rad)
            obs_j_local_x = dist_j * math.sin(angle_j_rad)
            obs_j_local_y = dist_j * math.cos(angle_j_rad)
            gap_width = math.hypot(obs_i_local_x - obs_j_local_x, obs_i_local_y - obs_j_local_y)
            is_navigable = gap_width >= config.GAP_MIN_WIDTH_CM
            gap_angle_local = (angle_i + angle_j) / 2.0
            gap_angle_global = q[2] + gap_angle_local
            while gap_angle_global > 180:
                gap_angle_global -= 360
            while gap_angle_global < -180:
                gap_angle_global += 360
            gap_info = {
                'left_sensor': i,
                'right_sensor': j,
                'gap_angle': gap_angle_global,
                'gap_width': gap_width,
                'is_navigable': is_navigable,
                'left_distance': dist_i,
                'right_distance': dist_j,
                'sensors_between': j - i - 1
            }
            gaps.append(gap_info)
            break
    return gaps

def ir_sensors_to_obstacles(q, ir_sensors):
    if not ir_sensors or len(ir_sensors) < 7:
        return []
    obstacles = []
    theta_robot_rad = math.radians(q[2])
    for i in range(7):
        ir_value = ir_sensors[i]
        if ir_value < config.IR_THRESHOLD_DETECT:
            continue
        d_estimate = ir_value_to_distance(ir_value, sensor_index=i)
        if i not in config.IR_SENSOR_ANGLES:
            continue
        sensor_angle_from_front_deg = config.IR_SENSOR_ANGLES[i]
        sensor_angle_from_front_rad = math.radians(sensor_angle_from_front_deg)
        sensor_direction_global = theta_robot_rad + sensor_angle_from_front_rad
        sensor_global_x = q[0] + config.IR_SENSOR_RADIUS * math.cos(sensor_direction_global)
        sensor_global_y = q[1] + config.IR_SENSOR_RADIUS * math.sin(sensor_direction_global)
        obs_x = sensor_global_x + d_estimate * math.cos(sensor_direction_global)
        obs_y = sensor_global_y + d_estimate * math.sin(sensor_direction_global)
        obstacles.append((obs_x, obs_y, ir_value))
    return obstacles

def find_best_free_direction(ir_sensors, current_heading_deg, goal_angle_deg):
    normalized_ir = [normalize_ir_reading(ir_sensors[i], i) for i in range(7)]
    sensor_angles = [config.IR_SENSOR_ANGLES.get(i, 0) for i in range(7)]
    freedom_scores = []
    for i in range(7):
        ir_norm = normalized_ir[i]
        if ir_norm < config.IR_THRESHOLD_DETECT:
            freedom = 1.0
        elif ir_norm >= config.IR_THRESHOLD_EMERGENCY:
            freedom = 0.0
        else:
            freedom = 1.0 - (ir_norm - config.IR_THRESHOLD_DETECT) / (config.IR_THRESHOLD_EMERGENCY - config.IR_THRESHOLD_DETECT)
        freedom_scores.append(freedom)

    error_to_goal = goal_angle_deg - current_heading_deg
    while error_to_goal > 180:
        error_to_goal -= 360
    while error_to_goal < -180:
        error_to_goal += 360

    best_score = -1000
    best_direction = 0
    min_freedom = 1.0
    for i in range(7):
        sensor_angle = sensor_angles[i]
        freedom = freedom_scores[i]
        angle_diff = abs(sensor_angle - error_to_goal)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        score = 0.7 * freedom - 0.3 * (angle_diff / 180.0)
        if score > best_score:
            best_score = score
            best_direction = sensor_angle
            min_freedom = min(min_freedom, freedom)

    avg_freedom = sum(freedom_scores) / len(freedom_scores)
    should_slow = avg_freedom < 0.5
    return best_direction, min_freedom, should_slow

def repulsive_force(q, ir_sensors, k_rep=None, d_influence=None, gaps=None):
    if k_rep is None:
        k_rep = config.K_REPULSIVE
    if d_influence is None:
        d_influence = config.D_INFLUENCE
    if not ir_sensors or max(ir_sensors) < config.IR_THRESHOLD_DETECT:
        return 0.0, 0.0

    fx_total = 0.0
    fy_total = 0.0
    theta_robot_rad = math.radians(q[2])

    for i in range(7):
        ir_value = ir_sensors[i]
        if ir_value < config.IR_THRESHOLD_DETECT:
            continue
        d_obstacle = ir_value_to_distance(ir_value, sensor_index=i)
        clearance = d_obstacle - config.ROBOT_RADIUS_CM
        if d_obstacle >= d_influence:
            continue
        d_safe = config.D_SAFE
        if clearance < 1.0:
            force_magnitude = k_rep * 10.0
        elif clearance < d_safe:
            term = (1.0 / clearance) - (1.0 / d_safe)
            force_magnitude = k_rep * (term ** 2)
        else:
            factor_alcance = 1.0 - (d_obstacle / d_influence)
            force_magnitude = k_rep * math.pow(d_safe / clearance, 3.0) * factor_alcance

        if gaps:
            for gap in gaps:
                if gap.get('is_navigable', False):
                    left_idx = gap.get('left_sensor', -1)
                    right_idx = gap.get('right_sensor', -1)
                    if i == left_idx or i == right_idx:
                        force_magnitude *= config.GAP_REPULSION_REDUCTION_FACTOR
                        break

        if i not in config.IR_SENSOR_ANGLES:
            continue
        sensor_angle_deg = config.IR_SENSOR_ANGLES[i]
        sensor_angle_rad = math.radians(sensor_angle_deg)
        sensor_direction_global = theta_robot_rad + sensor_angle_rad
        force_direction = sensor_direction_global + math.pi
        fx = force_magnitude * math.cos(force_direction)
        fy = force_magnitude * math.sin(force_direction)
        fx_total += fx
        fy_total += fy

    return fx_total, fy_total

def combined_potential_speeds(q, q_goal, ir_sensors=None, k_lin=None, k_ang=None, k_rep=None, d_influence=None, potential_type='linear'):
    global _last_v_linear
    if ir_sensors is None or not ir_sensors:
        return attractive_wheel_speeds(q, q_goal, k_lin=k_lin, k_ang=k_ang, potential_type=potential_type)

    if k_lin is None:
        if potential_type == 'linear':
            k_lin = config.K_LINEAR
        elif potential_type == 'quadratic':
            k_lin = config.K_QUADRATIC
        elif potential_type == 'conic':
            k_lin = config.K_CONIC
        elif potential_type == 'exponential':
            k_lin = config.K_EXPONENTIAL
        else:
            k_lin = config.K_LINEAR
    if k_ang is None:
        k_ang = config.K_ANGULAR
    if k_rep is None:
        k_rep = config.K_REPULSIVE
    if d_influence is None:
        d_influence = config.D_INFLUENCE

    normalized_ir = []
    if ir_sensors and len(ir_sensors) >= 7:
        for i in range(7):
            normalized_ir.append(normalize_ir_reading(ir_sensors[i], i))
    else:
        normalized_ir = ir_sensors if ir_sensors else []

    max_ir_all = 0
    max_ir_lateral = 0
    trapped_sensor_count = 0
    gaps = []
    navigable_gap_detected = False

    if normalized_ir and len(normalized_ir) >= 7:
        gaps = detect_navigable_gaps(normalized_ir, q)
        for gap in gaps:
            if gap.get('is_navigable', False):
                navigable_gap_detected = True
                break
        max_ir_all = max(normalized_ir)
        max_ir_lateral = max(normalized_ir[0], normalized_ir[6])
        if config.ENABLE_TRAP_ESCAPE and not navigable_gap_detected:
            for i in range(7):
                if normalized_ir[i] >= config.TRAP_DETECTION_IR_THRESHOLD:
                    trapped_sensor_count += 1

    is_trapped = (config.ENABLE_TRAP_ESCAPE and trapped_sensor_count >= config.TRAP_DETECTION_SENSOR_COUNT and not navigable_gap_detected)

    front_sensor_indices = [2, 3, 4]
    min_clearance_front = float('inf')
    min_distance_front = float('inf')
    for idx in front_sensor_indices:
        if normalized_ir[idx] >= config.IR_THRESHOLD_DETECT:
            dist = ir_value_to_distance(ir_sensors[idx], sensor_index=idx)
            clearance = dist - config.ROBOT_RADIUS_CM
            if clearance < min_clearance_front:
                min_clearance_front = clearance
            if dist < min_distance_front:
                min_distance_front = dist

    current_v = _last_v_linear if _last_v_linear > 0 else 8.0
    decel_rate = 20.0
    brake_distance = (current_v ** 2) / (2 * decel_rate)
    effective_clearance = min_clearance_front - brake_distance
    if effective_clearance < 5.0 or min_clearance_front < 3.0:
        v_max_allowed = config.V_MAX_EMERGENCY
        safety_level = "EMERGENCY"
    elif effective_clearance < 12.0 or min_clearance_front < 8.0:
        v_max_allowed = config.V_MAX_CRITICAL
        safety_level = "CRITICAL"
    elif effective_clearance < 20.0 or min_distance_front < 15.0:
        v_max_allowed = config.V_MAX_WARNING
        safety_level = "WARNING"
    elif effective_clearance < 30.0 or min_distance_front < 25.0:
        v_max_allowed = config.V_MAX_CAUTION
        safety_level = "CAUTION"
    else:
        v_max_allowed = config.V_MAX_CM_S
        safety_level = "CLEAR"

    if navigable_gap_detected:
        max_gap_width = max([g.get('gap_width', 0) for g in gaps if g.get('is_navigable', False)], default=0)
        if max_gap_width > config.ROBOT_DIAMETER_CM + 30:
            v_max_allowed = min(v_max_allowed * 1.3, config.V_MAX_CM_S)
        elif max_gap_width > config.ROBOT_DIAMETER_CM + 15:
            v_max_allowed = min(v_max_allowed * 1.15, config.V_MAX_CM_S)

    if is_trapped:
        safety_level = "TRAPPED"

    dx_goal = q_goal[0] - q[0]
    dy_goal = q_goal[1] - q[1]
    distance = math.hypot(dx_goal, dy_goal)
    k_lin_effective = k_lin
    k_rep_effective = k_rep

    if normalized_ir and len(normalized_ir) >= 7:
        max_frontal = max(normalized_ir[2], normalized_ir[3], normalized_ir[4])
        if max_frontal >= config.IR_THRESHOLD_CRITICAL:
            k_rep_effective = k_rep * 2.0
        elif max_frontal >= config.IR_THRESHOLD_WARNING:
            k_rep_effective = k_rep * 1.5

    if is_trapped:
        k_lin_effective = k_lin * config.TRAP_ATTRACTIVE_REDUCTION
        k_rep_effective = k_rep_effective * config.TRAP_REPULSIVE_BOOST

    if distance < config.TOL_DIST_CM:
        fx_att = 0.0
        fy_att = 0.0
    else:
        direction_x = dx_goal / distance
        direction_y = dy_goal / distance
        if potential_type == 'linear':
            f_magnitude = k_lin_effective * distance
        elif potential_type == 'quadratic':
            f_magnitude = k_lin_effective * (distance ** 2) / 10.0
        elif potential_type == 'conic':
            f_magnitude = k_lin_effective * min(distance, 100.0) * 2.0
        elif potential_type == 'exponential':
            f_magnitude = k_lin_effective * (1 - math.exp(-distance / 50.0)) * 20.0
        else:
            f_magnitude = k_lin_effective * distance
        fx_att = f_magnitude * direction_x
        fy_att = f_magnitude * direction_y

    fx_rep, fy_rep = repulsive_force(q, ir_sensors, k_rep=k_rep_effective, d_influence=d_influence, gaps=gaps)

    if distance < config.TOL_DIST_CM:
        v_base = 0.0
    else:
        if potential_type == 'linear':
            v_base = k_lin_effective * distance
        elif potential_type == 'quadratic':
            v_base = k_lin_effective * (distance ** 2) / 10.0
        elif potential_type == 'conic':
            v_base = k_lin_effective * min(distance, 100.0) * 2.0
        elif potential_type == 'exponential':
            v_base = k_lin_effective * (1 - math.exp(-distance / 50.0)) * 20.0
        else:
            v_base = k_lin_effective * distance
        v_base = min(v_base, v_max_allowed)
        v_base = min(v_base, config.V_MAX_CM_S)
        if is_trapped and v_base < config.TRAP_MIN_FORWARD_SPEED:
            v_base = config.TRAP_MIN_FORWARD_SPEED
        max_accel = config.ACCEL_RAMP_CM_S2 * config.CONTROL_DT
        if v_base > _last_v_linear + max_accel:
            v_base = _last_v_linear + max_accel
        _last_v_linear = v_base

    desired_angle_att = math.atan2(dy_goal, dx_goal)
    f_rep_mag = math.hypot(fx_rep, fy_rep)
    if f_rep_mag > 0.5:
        angle_rep = math.atan2(fy_rep, fx_rep)
        weight_rep = min(f_rep_mag / 3.5, 0.85)
        weight_att = 1.0 - weight_rep
        combined_x = weight_att * math.cos(desired_angle_att) + weight_rep * math.cos(angle_rep)
        combined_y = weight_att * math.sin(desired_angle_att) + weight_rep * math.sin(angle_rep)
        desired_angle = math.atan2(combined_y, combined_x)
        if weight_rep > 0.7:
            extra_slowdown = max(0.5, 1.0 - weight_rep * 0.4)
        elif weight_rep > 0.4:
            extra_slowdown = max(0.7, 1.0 - weight_rep * 0.3)
        else:
            extra_slowdown = max(0.85, 1.0 - weight_rep * 0.2)
        v_linear = v_base * extra_slowdown
    else:
        desired_angle = desired_angle_att
        v_linear = v_base

    theta_rad = math.radians(q[2])
    angle_error = _wrap_pi(desired_angle - theta_rad)

    if ir_sensors and len(ir_sensors) >= 7:
        max_left_lateral = max(ir_sensors[0], ir_sensors[1])
        max_right_lateral = max(ir_sensors[5], ir_sensors[6])
        if max_left_lateral >= config.IR_THRESHOLD_CRITICAL and angle_error > 0.3:
            angle_error = min(angle_error, 0.1)
        elif max_right_lateral >= config.IR_THRESHOLD_CRITICAL and angle_error < -0.3:
            angle_error = max(angle_error, -0.1)
        elif max_left_lateral >= config.IR_THRESHOLD_WARNING and angle_error > 0.5:
            angle_error *= 0.5
        elif max_right_lateral >= config.IR_THRESHOLD_WARNING and angle_error < -0.5:
            angle_error *= 0.5

    angle_factor = math.cos(angle_error)
    if distance > 50.0:
        min_factor = 0.6
    elif distance > 20.0:
        min_factor = 0.4
    else:
        min_factor = 0.2
    if angle_factor < min_factor:
        angle_factor = min_factor
    v_linear *= angle_factor

    if ir_sensors and len(ir_sensors) >= 7:
        lateral_indices = [0, 1, 5, 6]
        min_lateral_clearance = float('inf')
        for idx in lateral_indices:
            if ir_sensors[idx] >= config.IR_THRESHOLD_DETECT:
                dist = ir_value_to_distance(ir_sensors[idx], sensor_index=idx)
                clearance = dist - config.ROBOT_RADIUS_CM
                if clearance < min_lateral_clearance:
                    min_lateral_clearance = clearance
        if min_lateral_clearance < 5.0:
            v_linear *= 0.4
        elif min_lateral_clearance < 10.0:
            v_linear *= 0.65
        elif min_lateral_clearance < 15.0:
            v_linear *= 0.8

    if distance > 30.0 and v_linear < 8.0 and safety_level == "CLEAR":
        v_linear = 8.0

    k_ang_adjusted = k_ang
    if is_trapped:
        k_ang_adjusted = k_ang * config.TRAP_ANGULAR_BOOST
    elif max_ir_lateral >= config.IR_THRESHOLD_CRITICAL:
        k_ang_adjusted = k_ang * 1.5
    elif max_ir_lateral >= config.IR_THRESHOLD_WARNING:
        k_ang_adjusted = k_ang * 1.25

    if distance < 15.0 and max_ir_lateral < config.IR_THRESHOLD_CAUTION and not is_trapped:
        reduction_factor = 0.3 + 0.7 * ((distance - 5.0) / 10.0)
        reduction_factor = max(0.3, min(1.0, reduction_factor))
        k_ang_adjusted = k_ang_adjusted * reduction_factor

    omega = k_ang_adjusted * angle_error
    omega_max_rad_s = config.W_MAX_CM_S / (config.WHEEL_BASE_CM / 2.0)
    omega = max(-omega_max_rad_s, min(omega_max_rad_s, omega))

    half_base = config.WHEEL_BASE_CM / 2.0
    if distance > config.TOL_DIST_CM and v_linear > 0:
        if distance > 30.0:
            min_wheel_speed = 4.0
        elif distance > 10.0:
            min_wheel_speed = 2.0
        else:
            min_wheel_speed = 0.0
        max_omega_for_arc = (v_linear - min_wheel_speed) / half_base
        if abs(omega) > max_omega_for_arc:
            omega = math.copysign(max_omega_for_arc, omega)

    v_left = v_linear - half_base * omega
    v_right = v_linear + half_base * omega
    if distance > config.TOL_DIST_CM * 2 and v_linear > 0:
        if v_left < 0 or v_right < 0:
            max_omega_positive = v_linear / half_base
            if omega > max_omega_positive:
                omega = max_omega_positive * 0.95
            elif omega < -max_omega_positive:
                omega = -max_omega_positive * 0.95
            v_left = v_linear - half_base * omega
            v_right = v_linear + half_base * omega

    v_left = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_left))
    v_right = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_right))

    info = {
        'v_linear': v_linear,
        'omega': omega,
        'angle_error_deg': math.degrees(angle_error),
        'fx_attractive': fx_att,
        'fy_attractive': fy_att,
        'fx_repulsive': fx_rep,
        'fy_repulsive': fy_rep,
        'fx_total': fx_att + fx_rep,
        'fy_total': fy_att + fy_rep,
        'force_magnitude': math.hypot(fx_att + fx_rep, fy_att + fy_rep),
        'num_obstacles': len(ir_sensors_to_obstacles(q, ir_sensors)),
        'potential_type': potential_type,
        'safety_level': safety_level,
        'max_ir_all': max_ir_all,
        'is_trapped': is_trapped,
        'trapped_sensor_count': trapped_sensor_count,
        'max_ir_lateral': max_ir_lateral,
        'v_max_allowed': v_max_allowed,
        'num_gaps': len(gaps),
        'navigable_gap_detected': navigable_gap_detected,
        'gap_widths': [gap.get('gap_width', 0) for gap in gaps] if gaps else [],
        'gap_angles': [gap.get('gap_angle', 0) for gap in gaps] if gaps else []
    }

    return v_left, v_right, distance, info


