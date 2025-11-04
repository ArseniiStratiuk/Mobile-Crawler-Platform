import time
import sys
import threading
from pymavlink import mavutil

# --- КОНФІГУРАЦІЯ ---
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 57600

# Канали RC (з інформації від ментора)
THROTTLE_CHANNEL = 3  # 3 канал вперед-назад
STEER_CHANNEL = 1     # 1 канал вліво-вправо

# Значення PWM (з "default 1500")
NEUTRAL_PWM = 1500
FORWARD_PWM = 1700
BACKWARD_PWM = 1300
LEFT_PWM = 1300
RIGHT_PWM = 1700

# Кнопки передач (з інформації від ментора)
GEAR_UP_BIT = 2048    # bit 11
GEAR_DOWN_BIT = 4096  # bit 12

class RoverController:
    def __init__(self, port, baud):
        """Встановлює з'єднання з MAVLink."""
        print(f"Connecting to {port} at {baud} baud...")
        self.master = mavutil.mavlink_connection(
            port, 
            baud=baud, 
            source_system=255, 
            mavlink2=True
        )
        
        # Чекаємо на перший HEARTBEAT
        print("Waiting for vehicle heartbeat...")
        self.master.wait_heartbeat()
        print("Vehicle connected!")

        # Поточний стан керування
        self.current_gear = 0
        self.throttle_pwm = NEUTRAL_PWM
        self.steer_pwm = NEUTRAL_PWM
        
        self.running = True
        self.override_thread = threading.Thread(target=self._rc_override_loop, daemon=True)

    def _rc_override_loop(self):
        """
        Ця функція працює у фоновому потоці.
        Вона БЕЗПЕРЕРВНО надсилає стан джойстика (RC_CHANNELS_OVERRIDE).
        Це критично, інакше робот активує failsafe.
        """
        while self.running:
            # Створюємо масив з 18 каналів, заповнений NEUTRAL_PWM
            overrides = [NEUTRAL_PWM] * 18
            
            # Встановлюємо значення керма та газу
            overrides[STEER_CHANNEL - 1] = self.steer_pwm
            overrides[THROTTLE_CHANNEL - 1] = self.throttle_pwm
            
            # Надсилаємо повідомлення RC_CHANNELS_OVERRIDE (70)
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *overrides
            )
            time.sleep(0.1) # Надсилаємо 10 разів на секунду

    def _send_button_press(self, button_bitmask):
        """Надсилає ОДНЕ повідомлення MANUAL_CONTROL (69) для імітації натискання кнопки."""
        print(f"Sending button press: {button_bitmask}")
        # Натискаємо кнопку
        self.master.mav.manual_control_send(
            self.master.target_system,
            0, # x (pitch)
            0, # y (roll)
            0, # z (throttle)
            0, # r (yaw)
            button_bitmask # buttons
        )
        # Чекаємо 0.1с
        time.sleep(0.1)
        # Відпускаємо кнопку (надсилаємо 0)
        self.master.mav.manual_control_send(
            self.master.target_system, 0, 0, 0, 0, 0
        )
        print("Button released")

    def arm(self):
        """Армує робота."""
        print("Attempting to ARM...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # param1: 1 to arm
            0, 0, 0, 0, 0, 0
        )
        
        # Чекаємо на підтвердження
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
            print("ARMED successfully!")
            return True
        else:
            print(f"Failed to ARM! ACK: {ack}")
            return False

    def disarm(self):
        """Дизармує робота."""
        print("Attempting to DISARM...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # param1: 0 to disarm
            0, 0, 0, 0, 0, 0
        )
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"DISARMED. ACK: {ack}")

    def gear_up(self):
        if self.current_gear < 3:
            self._send_button_press(GEAR_UP_BIT)
            self.current_gear += 1
            print(f"Current Gear: {self.current_gear}")
        else:
            print("Already in highest gear (3)")

    def gear_down(self):
        if self.current_gear > 0:
            self._send_button_press(GEAR_DOWN_BIT)
            self.current_gear -= 1
            print(f"Current Gear: {self.current_gear}")
        else:
            print("Already in lowest gear (0)")

    def process_command(self, cmd):
        """Обробляє введені користувачем команди."""
        if cmd == 'w':
            print("Moving FORWARD")
            self.throttle_pwm = FORWARD_PWM
        elif cmd == 's':
            print("Moving BACKWARD")
            self.throttle_pwm = BACKWARD_PWM
        elif cmd == 'a':
            print("Turning LEFT")
            self.steer_pwm = LEFT_PWM
        elif cmd == 'd':
            print("Turning RIGHT")
            self.steer_pwm = RIGHT_PWM
        elif cmd == 'stop':
            print("STOPPING")
            self.throttle_pwm = NEUTRAL_PWM
            self.steer_pwm = NEUTRAL_PWM
        elif cmd == 'e':
            print("GEAR UP")
            self.gear_up()
        elif cmd == 'q':
            print("GEAR DOWN")
            self.gear_down()
        elif cmd == 'quit':
            print("Quitting...")
            self.running = False
        else:
            print("Unknown command")

    def start(self):
        """Головний цикл, який чекає на команди."""
        if not self.arm():
            self.running = False
            return
            
        # Запускаємо фоновий потік для RC_CHANNELS_OVERRIDE
        self.override_thread.start()
        
        print("\n--- Rover Control Ready ---")
        print(" w = Forward")
        print(" s = Backward")
        print(" a = Left")
        print(" d = Right")
        print(" e = Gear Up")
        print(" q = Gear Down")
        print(" stop = STOP")
        print(" quit = Disarm and Exit")
        print(" (Ctrl+D = Disarm and Exit)")
        print("---------------------------")
        
        try:
            while self.running:
                # Чекаємо на інструкцію від користувача
                cmd = input("Enter command: ").strip().lower()
                self.process_command(cmd)
                
        except EOFError:
            print("\nEOFError (Ctrl+D) detected. Quitting...")
            self.running = False
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt (Ctrl+C) detected. Quitting...")
            self.running = False

    def stop(self):
        """Зупиняє всі процеси та дизармить робота."""
        print("Stopping controller...")
        self.running = False
        if self.override_thread.is_alive():
            self.override_thread.join()
        
        # Встановлюємо нейтральні значення перед дизармом
        self.throttle_pwm = NEUTRAL_PWM
        self.steer_pwm = NEUTRAL_PWM
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *([NEUTRAL_PWM] * 18)
        )
        time.sleep(0.5)
        self.disarm()
        print("Controller stopped. Robot disarmed.")

# --- Головний блок запуску ---
if __name__ == "__main__":
    controller = RoverController(SERIAL_PORT, BAUD_RATE)
    try:
        controller.start()
    finally:
        # Цей блок 'finally' гарантує, що робот буде дизармлений
        # навіть якщо програма впаде з помилкою.
        controller.stop()
