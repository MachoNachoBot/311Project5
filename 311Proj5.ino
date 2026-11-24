#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define MY_TCCR1A (*(volatile unsigned char *)0x80)
#define MY_TCCR1B (*(volatile unsigned char *)0x81)
#define MY_TCNT1L (*(volatile unsigned char *)0x84)
#define MY_TCNT1H (*(volatile unsigned char *)0x85)
#define MY_ICR1L  (*(volatile unsigned char *)0x86)
#define MY_ICR1H  (*(volatile unsigned char *)0x87)
#define MY_OCR1AL (*(volatile unsigned char *)0x88)
#define MY_OCR1AH (*(volatile unsigned char *)0x89)

#define MY_DDRB   (*(volatile unsigned char *)0x24)
#define MY_PORTB  (*(volatile unsigned char *)0x25)
#define MY_PIND   (*(volatile unsigned char *)0x29)
#define MY_DDRD   (*(volatile unsigned char *)0x2A)
#define MY_PORTD  (*(volatile unsigned char *)0x2B)

#define MY_COM1A1     7
#define MY_COM1A0     6
#define MY_WGM11      1
#define MY_WGM13      4
#define MY_WGM12      3
#define MY_CS10       0

#define LED1_BIT      2
#define LED2_BIT      3
#define MOTOR_BIT     1
#define BUTTON_BIT    4

#define PRIO_SERIAL   1
#define PRIO_IO       2

#define STACK_LED     128
#define STACK_SERIAL  192
#define STACK_FAN     128

TickType_t msToTicks(unsigned long ms) {
  unsigned long ticks = (ms / 1000UL) * configTICK_RATE_HZ
                      + ((ms % 1000UL) * configTICK_RATE_HZ + 500UL) / 1000UL;
  if (ticks == 0) {
    ticks = 1;
  }
  if (ticks >= portMAX_DELAY) {
    ticks = portMAX_DELAY - 1;
  }
  return (TickType_t)ticks;
}

const unsigned int PWM_TOP = 799;

#define FAN_OFF   0
#define FAN_LOW   1
#define FAN_MED   2
#define FAN_HIGH  3
const unsigned char speedDuty[4] = {0, 40, 65, 100};

const unsigned char fanSequence[] = {FAN_OFF, FAN_LOW, FAN_MED, FAN_HIGH, FAN_MED, FAN_LOW};
const unsigned char SEQ_LEN = sizeof(fanSequence) / sizeof(fanSequence[0]);

const unsigned long KICK_MS = 150;
const unsigned char DEBOUNCE_TICKS = 2;

void writeTimer1Reg16(volatile unsigned char *high, volatile unsigned char *low, unsigned int value) {
  taskENTER_CRITICAL();
  *high = (unsigned char)(value >> 8);
  *low  = (unsigned char)(value & 0xFF);
  taskEXIT_CRITICAL();
}

void init_motor_pwm() {
  MY_PORTB &= ~(1 << MOTOR_BIT);
  MY_DDRB  |=  (1 << MOTOR_BIT);

  MY_TCCR1A = 0;
  MY_TCCR1B = 0;
  writeTimer1Reg16(&MY_TCNT1H, &MY_TCNT1L, 0);
  writeTimer1Reg16(&MY_ICR1H,  &MY_ICR1L,  PWM_TOP);
  writeTimer1Reg16(&MY_OCR1AH, &MY_OCR1AL, 0);

  MY_TCCR1A = (1 << MY_WGM11);
  MY_TCCR1B = (1 << MY_WGM13) | (1 << MY_WGM12) | (1 << MY_CS10);
}

void setMotorDuty(unsigned char percent) {
  if (percent == 0) {
    taskENTER_CRITICAL();
    MY_TCCR1A &= ~((1 << MY_COM1A1) | (1 << MY_COM1A0));
    MY_PORTB  &= ~(1 << MOTOR_BIT);
    taskEXIT_CRITICAL();
    return;
  }
  if (percent > 100) {
    percent = 100;
  }

  unsigned int ocr = (unsigned int)(((unsigned long)(PWM_TOP + 1) * percent) / 100);
  if (ocr > 0) {
    ocr -= 1;
  }
  writeTimer1Reg16(&MY_OCR1AH, &MY_OCR1AL, ocr);
  taskENTER_CRITICAL();
  MY_TCCR1A |= (1 << MY_COM1A1);
  taskEXIT_CRITICAL();
}

void init_button() {
  MY_DDRD  &= ~(1 << BUTTON_BIT);
  MY_PORTD |=  (1 << BUTTON_BIT);
}

unsigned char readButton() {
  return (MY_PIND >> BUTTON_BIT) & 0x01;
}

const unsigned char ledBits[2] = {LED1_BIT, LED2_BIT};

void init_leds() {
  for (int i = 0; i < 2; i++) {
    MY_PORTD &= ~(1 << ledBits[i]);
    MY_DDRD  |=  (1 << ledBits[i]);
  }
}

void writeLed(int i, unsigned char on) {
  taskENTER_CRITICAL();
  if (on) {
    MY_PORTD |= (1 << ledBits[i]);
  } else {
    MY_PORTD &= ~(1 << ledBits[i]);
  }
  taskEXIT_CRITICAL();
}

QueueHandle_t ledQueue[2];

void taskLed(void *param) {
  int i = (int)(size_t)param;
  TickType_t halfPeriod = 0;
  TickType_t lastToggle = xTaskGetTickCount();
  unsigned char on = 0;
  unsigned long newHalfMs;

  for (;;) {
    TickType_t wait;
    if (halfPeriod == 0) {
      wait = portMAX_DELAY;
    } else {
      TickType_t elapsed = xTaskGetTickCount() - lastToggle;
      wait = (elapsed >= halfPeriod) ? 0 : (TickType_t)(halfPeriod - elapsed);
    }

    if (xQueueReceive(ledQueue[i], &newHalfMs, wait) == pdTRUE) {
      halfPeriod = msToTicks(newHalfMs);
      on = 0;
      writeLed(i, 0);
      lastToggle = xTaskGetTickCount();
    } else {
      lastToggle += halfPeriod;
      on = !on;
      writeLed(i, on);
    }
  }
}

char rxBuffer[24];
int rxIndex = 0;
int targetLed = 0;
int promptStage = 0;

void processCommand(long parsedVal) {
  if (promptStage == 0) {
    if (parsedVal == 1 || parsedVal == 2) {
      targetLed = parsedVal - 1;
      promptStage = 1;
      Serial.println("What interval (in msec)?");
    } else {
      Serial.println("Invalid selection. What LED? (1 or 2)");
    }
  } else {
    if (parsedVal > 0) {
      unsigned long halfTime = parsedVal / 2;
      if (halfTime == 0) {
        halfTime = 1;
      }
      xQueueOverwrite(ledQueue[targetLed], &halfTime);

      promptStage = 0;
      Serial.println("What LED? (1 or 2)");
    } else {
      Serial.println("Invalid time. What interval (in msec)?");
    }
  }
}

void taskSerial(void *param) {
  (void)param;
  Serial.println("What LED? (1 or 2)");

  for (;;) {
    while (Serial.available() > 0) {
      char incoming = Serial.read();

      if (incoming == '\n' || incoming == '\r') {
        if (rxIndex > 0) {
          rxBuffer[rxIndex] = '\0';
          long parsedVal = atol(rxBuffer);
          rxIndex = 0;
          processCommand(parsedVal);
        }
      } else if (rxIndex < (int)sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = incoming;
      }
    }
    vTaskDelay(1);
  }
}

void taskFan(void *param) {
  (void)param;
  unsigned char stable = readButton();
  unsigned char lastRaw = stable;
  unsigned char sameCount = 0;
  unsigned char seqIndex = 0;
  unsigned char currentSpeed = FAN_OFF;
  unsigned char kicking = 0;
  TickType_t kickStart = 0;
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&wake, 1);

    if (kicking && (TickType_t)(xTaskGetTickCount() - kickStart) >= msToTicks(KICK_MS)) {
      kicking = 0;
      setMotorDuty(speedDuty[currentSpeed]);
    }

    unsigned char raw = readButton();

    if (raw != lastRaw) {
      lastRaw = raw;
      sameCount = 0;
    } else if (raw != stable) {
      if (++sameCount >= DEBOUNCE_TICKS) {
        stable = raw;
        sameCount = 0;
        if (stable == 0) {
          seqIndex = (seqIndex + 1) % SEQ_LEN;
          unsigned char speed = fanSequence[seqIndex];

          if (currentSpeed == FAN_OFF && speed != FAN_OFF) {
            setMotorDuty(100);
            kicking = 1;
            kickStart = xTaskGetTickCount();
          } else {
            kicking = 0;
            setMotorDuty(speedDuty[speed]);
          }
          currentSpeed = speed;
        }
      }
    }
  }
}

void setup() {
  init_motor_pwm();
  init_button();
  init_leds();

  Serial.begin(9600);

  ledQueue[0] = xQueueCreate(1, sizeof(unsigned long));
  ledQueue[1] = xQueueCreate(1, sizeof(unsigned long));

  xTaskCreate(taskLed,    "LED1",   STACK_LED,    (void *)0, PRIO_IO,     NULL);
  xTaskCreate(taskLed,    "LED2",   STACK_LED,    (void *)1, PRIO_IO,     NULL);
  xTaskCreate(taskFan,    "Fan",    STACK_FAN,    NULL,      PRIO_IO,     NULL);
  xTaskCreate(taskSerial, "Serial", STACK_SERIAL, NULL,      PRIO_SERIAL, NULL);
}

void loop() {
}
