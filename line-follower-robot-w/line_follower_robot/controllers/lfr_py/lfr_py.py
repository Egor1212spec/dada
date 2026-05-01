from controller import Robot
import math

# ─── Настройки ───────────────────────────────────────────────────────────────
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
FINISH_POINT = (-17.927, 6.051)
FINISH_RADIUS = 0.05
# ─── Массив целей (x, y) — заполни сам ──────────────────────────────────────
TARGETS = [
    (-7.549,4.551),
    # (x, y),
    # (x, y),
    # (x, y),
]

# Радиус попадания в цель (в метрах)
TARGET_RADIUS = 0.05

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

    # Состояния
    DRIVE       = 0
    ADV         = 1
    TRY_R       = 2
    GO_BACK     = 3
    UTURN       = 4
    TARGET_TURN = 5

    state       = DRIVE
    step_count  = 0
    cooldown    = 0
    junc_count  = 0

    # Одометрия
    x, y, theta = 0.0, 0.0, 0.0
    last_left_pos = 0.0
    last_right_pos = 0.0

    # Индекс текущей цели
    target_index = 0

    print("[BOT] Старт!")

    while robot.step(TIME_STEP) != -1:
        # ═══ Одометрия ════════════════════════════════════════════════════
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
# ═══ Проверка финиша ══════════════════════════════════════════════
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

        lv = 0.0
        rv = 0.0

        # ═══ Проверка целей ═══════════════════════════════════════════════
        if target_index < len(TARGETS):
            tx, ty = TARGETS[target_index]
            dist = math.sqrt((x - tx)**2 + (y - ty)**2)
            if dist < TARGET_RADIUS:
                target_index += 1
                print(f"цель {target_index} достигнута (x={x:.3f}, y={y:.3f})")
                state = TARGET_TURN
                step_count = 0
                junc_count = 0
                cooldown = 0
        # ═══ DRIVE ════════════════════════════════════════════════════════
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

        # ═══ ADV ══════════════════════════════════════════════════════════
        elif state == ADV:
            step_count += 1
            lv, rv = SPEED_FWD, SPEED_FWD
            if step_count >= ADVANCE:
                state = TRY_R
                step_count = 0

        # ═══ TRY_R ════════════════════════════════════════════════════════
        elif state == TRY_R:
            step_count += 1
            lv, rv = SPEED_TURN, -SPEED_TURN
            if step_count > MIN_ROT and center_on:
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN
            elif step_count >= TRY_R_LIMIT:
                state = GO_BACK
                step_count = 0

        # ═══ GO_BACK ══════════════════════════════════════════════════════
        elif state == GO_BACK:
            step_count += 1
            lv, rv = -SPEED_TURN, SPEED_TURN
            if step_count > 10 and center_on:
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN
            elif step_count >= 100:
                state = DRIVE
                step_count = 0

        # ═══ UTURN ════════════════════════════════════════════════════════
        elif state == UTURN:
            step_count += 1
            lv, rv = SPEED_TURN, -SPEED_TURN
            if step_count > UTURN_MIN and center_on:
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN
            elif step_count > UTURN_MAX:
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

        # ═══ TARGET_TURN: разворот 180° + выезд ══════════════════════════
        elif state == TARGET_TURN:
            step_count += 1
            if step_count <= 52:
                lv, rv = SPEED_TURN, -SPEED_TURN
            elif step_count <= 122:
                lv, rv = SPEED_FWD, SPEED_FWD
            else:
                if center_on or left_on or right_on:
                    state = DRIVE
                    step_count = 0
                    cooldown = COOLDOWN
                else:
                    lv, rv = SPEED_SLOW, SPEED_TURN
                if step_count > 250:
                    state = DRIVE
                    step_count = 0
                    cooldown = COOLDOWN

        lv = max(-MAX_SPEED, min(MAX_SPEED, lv))
        rv = max(-MAX_SPEED, min(MAX_SPEED, rv))
        left_motor.setVelocity(lv)
        right_motor.setVelocity(rv)

        # Отладка — только x, y
        if int(robot.getTime() * 1000) % 960 < TIME_STEP:
            names = ['DRIVE', 'ADV', 'TRY_R', 'GOBACK', 'UTURN', 'TGTURN']
            sname = names[state] if state < len(names) else str(state)
            print(f"[DBG] x={x:.3f} y={y:.3f} | {sname:<6} cd={cooldown:2d} s={step_count:3d}")

if __name__ == "__main__":
    run()