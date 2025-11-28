# **Mayze: Tilt-Controlled Maze Platform**
> Made by: Amar Aly, b00096144 | Ahmad Abu Srour b00097269 | Ahmad Aoude b00097433

Mayze is a real-time embedded game where a player tilts a physical maze using a joystick.  
The maze platform is driven by dual-axis servos, controlled by an STM32 microcontroller running FreeRTOS.  
A photoresistor detects when the ball reaches the goal, and the buzzer sounds to celebrate the win.

---

## 🚀 **Features**

- **Tilt-based control** using an analog joystick  
- **Dual-axis servo actuation** with cross-axis compensation  
- **Real-time concurrency** using FreeRTOS tasks  
- **Win detection** via a photoresistor  
- **Victory buzzer feedback**  
- **Mutex-synchronized shared state** between tasks  

---

## 🧩 **System Architecture**

### **FreeRTOS Tasks**

#### **1. Joystick Task**
- Reads analog joystick X/Y values  
- Updates a mutex-protected shared struct  
- Provides clean servo angle targets  

#### **2. Servo Task**
- Consumes joystick data from shared struct  
- Controls X-axis and Y-axis servos  
- Includes **cross-axis compensation**, which reduced effective rotation range  
- Moves the physical maze platform accordingly  

#### **3. Printer Task**
- Reads photoresistor value  
- Determines if the maze is solved  
- Prints win messages  
- Sets the `buzzer_flag`  

#### **4. Buzzer Task**
- Checks `buzzer_flag`  
- Activates the buzzer when signaled by the printer task  

---

## ⚙️ **Hardware Components**

- STM32 microcontroller (e.g., **STM32L476RG**)  
- Two servo motors (e.g., **SG90 micro servos**)  
- Analog 2-axis joystick  
- Photoresistor + voltage divider  
- Passive buzzer  
- Cardboard Maze

---

## 🔗 **Shared Memory (Mutex-Protected)**

All tasks interact through a single shared struct:

```c
typedef struct {
    uint32_t joystick_x;
    uint32_t joystick_y;
    uint16_t servo_x_pulse;
    uint16_t servo_y_pulse;
    uint8_t  buzzer_active;
} SharedData_t;
```

This struct is locked/unlocked via a FreeRTOS mutex to prevent race conditions.

---

## 🎮 **How the Game Works**

1. Player tilts the joystick  
2. Joystick task writes values → shared state  
3. Servo task updates platform tilt  
4. Ball rolls through the maze  
5. Photoresistor detects goal → win  
6. Printer task prints status + enables buzzer  
7. Buzzer task plays victory tone  

---

## 🧱 **Mechanical Challenges**

- Maze platform exhibited **instability** due to uneven load and servo wrong direction  
- Cross-axis compensation reduced the available servo range  
- Limited torque from SG90 servos caused drift under load  
- Required manual calibration to stabilize endpoints  

---

## 📈 **Future Improvements**

- Better stabilization for smoother servo motion  
- Reinforced mechanical platform   
- Adjustable difficulty modes  
- Improved cross-axis compensation mapping  

---
