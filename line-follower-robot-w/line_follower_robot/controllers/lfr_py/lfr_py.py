from controller import Robot

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
MIN_ROT     = 12   
TRY_R_LIMIT = 28   

COOLDOWN    = 35   
UTURN_MIN   = 50   
UTURN_MAX   = 140  

# Сколько шагов разворачиваться на ~180°
TARGET_TURN_STEPS = 52
# Сколько шагов уезжать от квадрата после разворота
TARGET_ESCAPE_STEPS = 30

def run():
    robot = Robot()

    left_motor  = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    sensors = []
    for name in ['gs0', 'gs1', 'gs2']:
        s = robot.getDevice(name)
        s.enable(TIME_STEP)
        sensors.append(s)

    camera = robot.getDevice('camera')
    camera.enable(TIME_STEP)

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
    target_cooldown = 0

    # ── НОВОЕ: запоминаем, какой цвет сейчас обрабатываем ──
    current_target_color = 0

    print("[BOT] Старт! Правило правой руки (v11 - Фикс зависания на квадрате)")

    while robot.step(TIME_STEP) != -1:
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

        # ═══ Определение цвета под камерой ════════════════════════════════
        # ВАЖНО: определяем цвет только когда НЕ крутимся на квадрате,
        # чтобы не получить ложное срабатывание во время TARGET_TURN.
        target = 0
        img = camera.getImageArray()
        if img:
            w, h = camera.getWidth(), camera.getHeight()
            r_sum = g_sum = b_sum = 0
            count = 0
            for dx in range(-2, 3):
                for dy in range(-4, 0):
                    px = img[w//2 + dx][h + dy]
                    r_sum += px[0]
                    g_sum += px[1]
                    b_sum += px[2]
                    count += 1
            avg_r = r_sum / count
            avg_g = g_sum / count
            avg_b = b_sum / count
            
            if avg_r > avg_g * 1.5 and avg_r > avg_b * 1.5 and avg_r > 30:
                target = 1
            elif avg_g > avg_r * 1.5 and avg_g > avg_b * 1.5 and avg_g > 30:
                target = 2
            elif avg_b > avg_r * 1.5 and avg_b > avg_g * 1.5 and avg_b > 30:
                target = 3
                
            if target == 0:
                black_count = 0
                for dx in [-20, -10, 0, 10, 20]:
                    if 0 <= w//2 + dx < w:
                        px = img[w//2 + dx][h - 2]
                        pr, pg, pb = px[0], px[1], px[2]
                        if pr < 80 and pg < 80 and pb < 80:
                            if pg < pr * 1.5 + 15 and pr < pg * 1.5 + 15 and pb < pr * 1.5 + 15:
                                black_count += 1
                if black_count >= 4:
                    target = 4

        # ═══ Реакция на цель — только из состояния DRIVE ══════════════════
        if target != 0 and state == DRIVE and target_cooldown == 0:
            if target == 4:
                print("работа окончена")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                break
            else:
                print(f"цель {target} достигнута")
                current_target_color = target
                state = TARGET_TURN
                step_count = 0
                junc_count = 0
                # Большой кулдаун — чтобы после выезда не среагировать снова
                target_cooldown = 200

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
                print("[BOT] Перекрёсток → проезд вперёд")
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
                print("[BOT] Центр → пробуем направо")
                state = TRY_R
                step_count = 0

        # ═══ TRY_R ════════════════════════════════════════════════════════
        elif state == TRY_R:
            step_count += 1
            lv, rv = SPEED_TURN, -SPEED_TURN

            if step_count > MIN_ROT and center_on:
                print(f"[BOT] Правый путь ЕСТЬ (шаг {step_count}) → DRIVE")
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

            elif step_count >= TRY_R_LIMIT:
                print(f"[BOT] Правого пути НЕТ → крутимся обратно")
                state = GO_BACK
                step_count = 0

        # ═══ GO_BACK ══════════════════════════════════════════════════════
        elif state == GO_BACK:
            step_count += 1
            lv, rv = -SPEED_TURN, SPEED_TURN

            if step_count > 10 and center_on:
                print(f"[BOT] Нашли путь ПРЯМО/НАЛЕВО → DRIVE")
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

            if step_count > UTURN_MIN and center_on and target == 0:
                print("[BOT] Разворот готов → DRIVE")
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

            elif step_count > UTURN_MAX:
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

        # ═══ TARGET_TURN: разворот + уезд от квадрата ════════════════════
        elif state == TARGET_TURN:
            step_count += 1

            if step_count <= TARGET_TURN_STEPS:
                # Фаза 1: разворот на ~180°
                lv, rv = SPEED_TURN, -SPEED_TURN

            elif step_count <= TARGET_TURN_STEPS + TARGET_ESCAPE_STEPS:
                # Фаза 2: едем вперёд, пока не найдём линию или не выйдет лимит
                lv, rv = SPEED_FWD, SPEED_FWD
                # ── КЛЮЧЕВОЕ ИСПРАВЛЕНИЕ ──────────────────────────────────
                # Как только датчики нашли линию — сразу переходим в DRIVE.
                # Не ждём конца таймера — это убирает зависание на квадрате.
                if center_on or left_on or right_on:
                    print("[BOT] Линия найдена после разворота → DRIVE")
                    state = DRIVE
                    step_count = 0
                    cooldown = COOLDOWN

            else:
                # Фаза 3: лимит вышел, всё равно едем дальше
                print("[BOT] Разворот после цели завершен → DRIVE")
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

        lv = max(-MAX_SPEED, min(MAX_SPEED, lv))
        rv = max(-MAX_SPEED, min(MAX_SPEED, rv))
        left_motor.setVelocity(lv)
        right_motor.setVelocity(rv)

        if int(robot.getTime() * 1000) % 960 < TIME_STEP:
            names = ['DRIVE', 'ADV', 'TRY_R', 'GOBACK', 'UTURN', 'TGT_TURN']
            sname = names[state] if state < len(names) else str(state)
            print(f"[DBG] gs=({raw[0]:.0f},{raw[1]:.0f},{raw[2]:.0f}) "
                  f"| {sname:<8} cd={cooldown:2d} s={step_count:3d} "
                  f"| vel=({lv:.2f},{rv:.2f})")

if __name__ == "__main__":
    run()