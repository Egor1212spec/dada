from controller import Robot
import math

TIME_STEP  = 32
MAX_SPEED  = 6.28
THRESHOLD  = 600

SPEED_FWD  = MAX_SPEED * 0.4
SPEED_FAST = MAX_SPEED * 0.55
SPEED_SLOW = MAX_SPEED * 0.10
SPEED_REV  = -MAX_SPEED * 0.18
SPEED_TURN = MAX_SPEED * 0.42

DEBOUNCE    = 4
ADVANCE     = 18
MIN_ROT     = 18
TRY_R_LIMIT = 28

COOLDOWN    = 35
UTURN_MIN   = 50
UTURN_MAX   = 140

WHEEL_RADIUS = 0.0205
AXLE_LENGTH  = 0.052

TARGETS = [
    (-6.803, -1.980),
    (-5.296, -1.978),
    (-6.261, 4.141),
]

TARGET_RADIUS = 0.15
FINISH_POINT = (-17.812, 4.287)
FINISH_RADIUS = 0.15

JUNCTION_RADIUS = 0.6

def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def snap_direction(theta):
    deg = math.degrees(normalize_angle(theta)) % 360
    if deg < 0:
        deg += 360
    if deg < 45 or deg >= 315:
        return 0
    elif deg < 135:
        return 90
    elif deg < 225:
        return 180
    else:
        return 270

def run():
    robot = Robot()

    left_motor  = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    left_ps = robot.getDevice('left wheel sensor')
    right_ps = robot.getDevice('right wheel sensor')
    left_ps.enable(TIME_STEP)
    right_ps.enable(TIME_STEP)

    sensors = []
    for name in ['gs0', 'gs1', 'gs2']:
        s = robot.getDevice(name)
        s.enable(TIME_STEP)
        sensors.append(s)

    DRIVE       = 0
    ADV         = 1
    SCAN_L      = 2
    SCAN_BACK_L = 3
    SCAN_R      = 4
    SCAN_BACK_R = 5
    CHOOSE      = 6
    UTURN       = 7
    TARGET_TURN = 8
    TURN_L      = 100
    TURN_R      = 101

    state       = DRIVE
    step_count  = 0
    cooldown    = 0
    junc_count  = 0

    x, y, theta = 0.0, 0.0, 0.0
    last_left_pos = 0.0
    last_right_pos = 0.0

    visited_targets = set()
    target_cooldown = 0

    has_left = False
    has_right = False
    has_forward = False
    steps_turned = 0
    line_lost = False

    returning = False
    last_turn = None

    while robot.step(TIME_STEP) != -1:
        left_pos = left_ps.getValue()
        right_pos = right_ps.getValue()

        dl = (left_pos - last_left_pos) * WHEEL_RADIUS
        dr = (right_pos - last_right_pos) * WHEEL_RADIUS

        d_theta = (dr - dl) / AXLE_LENGTH
        d_center = (dl + dr) / 2.0

        x += d_center * math.cos(theta + d_theta / 2.0)
        y += d_center * math.sin(theta + d_theta / 2.0)
        theta += d_theta

        last_left_pos = left_pos
        last_right_pos = right_pos

        fd = math.sqrt((x - FINISH_POINT[0])**2 + (y - FINISH_POINT[1])**2)
        if fd < FINISH_RADIUS:
            print("работа окончена")
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            break

        raw = [sensors[i].getValue() for i in range(3)]
        left_on   = raw[0] < THRESHOLD
        center_on = raw[1] < THRESHOLD
        right_on  = raw[2] < THRESHOLD

        if cooldown > 0:
            cooldown -= 1
        if target_cooldown > 0:
            target_cooldown -= 1

        lv = 0.0
        rv = 0.0

        if target_cooldown == 0 and state != TARGET_TURN:
            for i in range(len(TARGETS)):
                if i in visited_targets:
                    continue
                tx, ty = TARGETS[i]
                dist = math.sqrt((x - tx)**2 + (y - ty)**2)
                if dist < TARGET_RADIUS:
                    visited_targets.add(i)
                    print(f"цель {len(visited_targets)} достигнута (x={x:.3f}, y={y:.3f})")
                    state = TARGET_TURN
                    step_count = 0
                    junc_count = 0
                    cooldown = 0
                    target_cooldown = 300
                    break

        if state == DRIVE:
            any_line = left_on or center_on or right_on

            if left_on and right_on and cooldown == 0:
                junc_count += 1
            else:
                junc_count = 0

            if not any_line:
                state = UTURN
                step_count = 0
                junc_count = 0

            elif junc_count >= DEBOUNCE:
                state = ADV
                step_count = 0
                junc_count = 0

            else:
                if center_on and not left_on and not right_on:
                    lv, rv = SPEED_FWD, SPEED_FWD
                elif left_on and center_on and not right_on:
                    lv, rv = SPEED_SLOW, SPEED_FAST
                elif right_on and center_on and not left_on:
                    lv, rv = SPEED_FAST, SPEED_SLOW
                elif left_on and not center_on and not right_on:
                    lv, rv = SPEED_REV, SPEED_FAST
                elif right_on and not center_on and not left_on:
                    lv, rv = SPEED_FAST, SPEED_REV
                else:
                    lv, rv = SPEED_FWD, SPEED_FWD

        elif state == ADV:
            step_count += 1
            lv, rv = SPEED_FWD, SPEED_FWD
            if step_count >= ADVANCE:
                has_forward = center_on
                has_left = False
                has_right = False
                line_lost = False
                state = SCAN_L
                step_count = 0

        elif state == SCAN_L:
            step_count += 1
            lv, rv = -SPEED_TURN, SPEED_TURN
            if not center_on:
                line_lost = True
            if line_lost and center_on and step_count > 5:
                has_left = True
                steps_turned = step_count
                state = SCAN_BACK_L
                step_count = 0
            elif step_count >= TRY_R_LIMIT:
                has_left = False
                steps_turned = step_count
                state = SCAN_BACK_L
                step_count = 0

        elif state == SCAN_BACK_L:
            step_count += 1
            lv, rv = SPEED_TURN, -SPEED_TURN
            if step_count >= steps_turned:
                line_lost = False
                state = SCAN_R
                step_count = 0

        elif state == SCAN_R:
            step_count += 1
            lv, rv = SPEED_TURN, -SPEED_TURN
            if not center_on:
                line_lost = True
            if line_lost and center_on and step_count > 5:
                has_right = True
                steps_turned = step_count
                state = SCAN_BACK_R
                step_count = 0
            elif step_count >= TRY_R_LIMIT:
                has_right = False
                steps_turned = step_count
                state = SCAN_BACK_R
                step_count = 0

        elif state == SCAN_BACK_R:
            step_count += 1
            lv, rv = -SPEED_TURN, SPEED_TURN
            if step_count >= steps_turned:
                state = CHOOSE
                step_count = 0

        elif state == CHOOSE:
            chosen_rel = None

            if returning and last_turn is not None:
                if last_turn == 'left':
                    chosen_rel = 'left'
                elif last_turn == 'right':
                    chosen_rel = 'right'
                else:
                    chosen_rel = 'forward'
                returning = False
            else:
                if has_left:
                    chosen_rel = 'left'
                elif has_right:
                    chosen_rel = 'right'
                elif has_forward:
                    chosen_rel = 'forward'
                
                if chosen_rel is not None:
                    last_turn = chosen_rel

            if chosen_rel is not None:
                if chosen_rel == 'left':
                    line_lost = False
                    state = TURN_L
                elif chosen_rel == 'right':
                    line_lost = False
                    state = TURN_R
                else:
                    state = DRIVE
                    cooldown = COOLDOWN
                step_count = 0
            else:
                state = UTURN
                step_count = 0

        elif state == TURN_L:
            step_count += 1
            lv, rv = -SPEED_TURN, SPEED_TURN
            if not center_on:
                line_lost = True
            if line_lost and center_on and step_count > 5:
                print("[BOT] Повернули налево -> DRIVE")
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN
            elif step_count >= 80:
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

        elif state == TURN_R:
            step_count += 1
            lv, rv = SPEED_TURN, -SPEED_TURN
            if not center_on:
                line_lost = True
            if line_lost and center_on and step_count > 5:
                print("[BOT] Повернули направо -> DRIVE")
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN
            elif step_count >= 80:
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

        elif state == UTURN:
            step_count += 1
            lv, rv = SPEED_TURN, -SPEED_TURN
            if step_count > UTURN_MIN and center_on:
                returning = True
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN
            elif step_count > UTURN_MAX:
                returning = True
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

        elif state == TARGET_TURN:
            step_count += 1
            if step_count <= 52:
                lv, rv = SPEED_TURN, -SPEED_TURN
            elif step_count <= 130:
                lv, rv = SPEED_FWD, SPEED_FWD
            else:
                if center_on or left_on or right_on:
                    returning = True
                    state = DRIVE
                    step_count = 0
                    cooldown = COOLDOWN
                else:
                    lv, rv = SPEED_SLOW, SPEED_TURN
                if step_count > 250:
                    returning = True
                    state = DRIVE
                    step_count = 0
                    cooldown = COOLDOWN

        lv = max(-MAX_SPEED, min(MAX_SPEED, lv))
        rv = max(-MAX_SPEED, min(MAX_SPEED, rv))
        left_motor.setVelocity(lv)
        right_motor.setVelocity(rv)

if __name__ == "__main__":
    run()