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

# ВАЖНО: Лимит шагов! 28 шагов = ~100 градусов. 
# Этого хватит для правого поворота, но не даст уйти на 180° к старой линии!
TRY_R_LIMIT = 28   

COOLDOWN    = 35   
UTURN_MIN   = 50   
UTURN_MAX   = 140  

def run():
    robot = Robot()

    left_motor  = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    sensors =[]
    for name in ['gs0', 'gs1', 'gs2']:
        s = robot.getDevice(name)
        s.enable(TIME_STEP)
        sensors.append(s)

    # Состояния
    DRIVE    = 0   
    ADV      = 1   
    TRY_R    = 2   
    GO_BACK  = 3   
    UTURN    = 4   

    state       = DRIVE
    step_count  = 0
    cooldown    = 0
    junc_count  = 0

    print("[BOT] Старт! Правило правой руки (v10 - Исправлено)")

    while robot.step(TIME_STEP) != -1:
        raw =[sensors[i].getValue() for i in range(3)]
        left_on   = raw[0] < THRESHOLD
        center_on = raw[1] < THRESHOLD
        right_on  = raw[2] < THRESHOLD

        if cooldown > 0:
            cooldown -= 1

        lv = 0.0
        rv = 0.0

        # ═══ DRIVE: следование по линии ═══════════════════════════════════
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

        # ═══ ADV: проезд вперёд до центра перекрёстка ═════════════════════
        elif state == ADV:
            step_count += 1
            lv, rv = SPEED_FWD, SPEED_FWD

            if step_count >= ADVANCE:
                print("[BOT] Центр → пробуем направо")
                state = TRY_R
                step_count = 0

        # ═══ TRY_R: попытка повернуть направо ════════════════════════════
        elif state == TRY_R:
            step_count += 1
            lv, rv = SPEED_TURN, -SPEED_TURN

            if step_count > MIN_ROT and center_on:
                print(f"[BOT] Правый путь ЕСТЬ (шаг {step_count}) → DRIVE")
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

            # Защита от разворота на 180! Если шагов стало 28, а линии нет - прекращаем.
            elif step_count >= TRY_R_LIMIT:
                print(f"[BOT] Правого пути НЕТ (остановка на шаге {step_count}) → крутимся обратно")
                state = GO_BACK
                step_count = 0

        # ═══ GO_BACK: возврат (поиск пути ПРЯМО или НАЛЕВО) ══════════════
        elif state == GO_BACK:
            step_count += 1
            lv, rv = -SPEED_TURN, SPEED_TURN  # Вращение ВЛЕВО

            # Ищем линию (через 10 шагов, чтобы не зацепить мусор)
            if step_count > 10 and center_on:
                print(f"[BOT] Нашли путь ПРЯМО/НАЛЕВО → DRIVE")
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN
                
            elif step_count >= 100:
                state = DRIVE
                step_count = 0

        # ═══ UTURN: разворот при тупике ═══════════════════════════════════
        elif state == UTURN:
            step_count += 1
            lv, rv = SPEED_TURN, -SPEED_TURN

            if step_count > UTURN_MIN and center_on:
                print("[BOT] Разворот готов → DRIVE")
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

            elif step_count > UTURN_MAX:
                state = DRIVE
                step_count = 0
                cooldown = COOLDOWN

        lv = max(-MAX_SPEED, min(MAX_SPEED, lv))
        rv = max(-MAX_SPEED, min(MAX_SPEED, rv))
        left_motor.setVelocity(lv)
        right_motor.setVelocity(rv)

        # Отладка
        if int(robot.getTime() * 1000) % 960 < TIME_STEP:
            names =['DRIVE', 'ADV', 'TRY_R', 'GOBACK', 'UTURN']
            print(f"[DBG] gs=({raw[0]:.0f},{raw[1]:.0f},{raw[2]:.0f}) "
                  f"| {names[state]:<6} cd={cooldown:2d} s={step_count:3d} "
                  f"| vel=({lv:.2f},{rv:.2f})")

if __name__ == "__main__":
    run()