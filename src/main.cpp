#include <Arduino.h>
#include "GyverButton.h"
#define DEBUG // Расскоментировать для монитора порта
/*
Код ардуино: линейное движение с помощью шагового двигателя nemo 17 через драйвер тмс 2208
в сторону "а" и в сторону "б" приводится с помощью  нажатия кнопок "1" и "2" соответственно.
При движении в сторону "а" и срабатывании концевика "3" происходит остановка движения,
и после отпускания кнопки "1" через 2 секунд двигатель начинает крутиться в обратную сторону
(т.е. Движение в сторону "б")  до срабатывания концевика "4" . Либо до повторного нажатия кноки "1" или "2". 
Небходимо что бы скорость движения в сторону "а" регулировалась с помощью переменного резистора в любой момент работы кода.
А скорость движения в сторону "б" была фиксированной.
При зажатом концевике "3" движение разрешено только в сторону "б" при зажатом "4" только в сторону "а".
*/

/***********************Пины***********************/
// Пины двигателя
#define PIN_STEP 4   // Пин STEP
#define PIN_DIR 5    // Пин dir
#define PIN_ENABLE 6 // Пин enable, возможно он называется set

// Пины кнопок управления
#define PIN_BUTTON_A 12 // Пин кнопки движения к концевику A
#define PIN_BUTTON_B 7  // Пин кнопки движения к концевику B

// Пины концевиков
#define PIN_END_A 11 // Пин концевика A
#define PIN_END_B 9  // Пин концевика B

// Потенциометр (Крутилка)
#define PIN_REGULATOR A0 // Пин регулятора скорости

/**************************Настройки**************************/
// Скорость
#define SPEED_A_MIN 300      // Минимальная скорость в сторону А (Шагов в секунду)
#define SPEED_A_MAX 1500     // Максимальная скорость в сторону A
#define SPEED_B 1000         // Постоянная скорость в сторону B
#define TIMER_AUTO_TO_B 1000 // Задержка перед автостартом в сторону А

// Двигатель
#define ENABLE_SIGNAL LOW // Какой сигнал enable разрешает движение: HIGH (+5v) или LOW (GND)
#define INVERT_DIR true   // если мотор крутится не туда, то меняем значение true/false
#define GS_NO_ACCEL       // Эта строчка убирает плавный старт и плавный тормоз моторов, делая их резкими
#define ADDWORK_TIME 200  // Время в миллисекундах, в течение которого двигатель не выключеается

// Кнопки управления
#define DEBOUNCE_A_B 150 // устранение ложного срабатывания кнопок A и B (больше = медленнее и точнее)
/* Супер-мега настройки кнопок управления:
Ниже произойдёт инициализация кнопок, в скобках там три параметра (1, 2, 3)
1 - Пин кнопки
2 - LOW/HIGH - куда замыкается кнопка: LOW = GND, HIGH = +5v
3 - NORM_OPEN/NORM_CLOSE
 NORM_OPEN - в обычном состоянии кнопка отжата/разомкнута. (Обычно так)
 NORM_CLOSE - в обычном состоянии кнопка нажата/замкнута.
*/
GButton btnA(PIN_BUTTON_A, LOW, NORM_OPEN);
GButton btnB(PIN_BUTTON_B, LOW, NORM_OPEN);

// Концевики
#define END_A_CONNECTION LOW // тип подключения концевика А: LOW (к земле) / HIGH (+5v)
#define END_A_TYPE NORM_OPEN // тип концевика А NORM_OPEN/NORM_CLOSE (см. инструкцию на 7 строк выше)

#define END_B_CONNECTION LOW // тип подключения концевика B: LOW (к земле) / HIGH (+5v)
#define END_B_TYPE NORM_OPEN // тип концевика B NORM_OPEN/NORM_CLOSE (см. инструкцию на 10 строк выше)

#define DEBOUNCE_END 150 // устранение ложного срабатывания концевика

// Потенциометр
#define DEBOUNCE_REGULATOR 2   // устранение ложного срабатывания: чем больше, тем стабильнее работает крутилка
#define REGULATOR_INVERT false // true/false сменить направление регулятора

#define PROTECT 0 // защита от неверного подключения (true/false)

/****************************КОД*****************************/
#include "GyverStepper2.h"
#ifdef DEBUG
#define DD(x) Serial.println(x)
#define DDD(x) Serial.print(x)
#else
#define DD(x)
#define DDD(x)
#endif
GStepper2<STEPPER2WIRE> stepper(0, PIN_STEP, PIN_DIR, PIN_ENABLE); // Шаговый двигатель

#define END_A_TRUE END_A_CONNECTION ^ END_A_TYPE
#define END_B_TRUE END_B_CONNECTION ^ END_B_TYPE

#if END_A_CONNECTION
#define END_A_PINMODE INPUT
#else
#define END_A_PINMODE INPUT_PULLUP
#endif

#if END_B_CONNECTION
#define END_B_PINMODE INPUT
#else
#define END_B_PINMODE INPUT_PULLUP
#endif
class endBtn
{
private:
  byte _pin, _truth;
  bool _state;
  unsigned long _debounce = 0;

public:
  endBtn(byte pin, byte pin_mode, byte truth);
  bool state();
  // ~endBtn();
};

endBtn::endBtn(byte pin, byte pin_mode, byte truth)
{
  pinMode(pin, pin_mode);
  _pin = pin;
  _truth = truth;
}
bool endBtn::state()
{
  if (_state)
  {
    _state = (digitalRead(_pin) == _truth);
    if (_state)
    {
      return _state; // 1
    }
    _debounce = millis() + DEBOUNCE_END;
    return _state; // 0
  }

  if (_debounce)
  {
    if (millis() >= _debounce)
    {
      _debounce = 0;
      _state = (digitalRead(_pin) == _truth);
    }
    return _state;
  }
  _state = (digitalRead(_pin) == _truth);
  return _state;
}
endBtn endA(PIN_END_A, END_A_PINMODE, END_A_TRUE);
endBtn endB(PIN_END_B, END_B_PINMODE, END_B_TRUE);

bool f_onA = false, f_onB = false, f_start = false, f_work = 0;
uint16_t regulator = 0;
auto speed = SPEED_A_MAX;

/**
 * @brief Определяем скорость в глобальную переменную speed
 *
 * @return true - была смена скорости;
 * @return false - скорость не меняли
 */
bool check_regulator()
{
#if REGULATOR_INVERT
  uint16_t regulator_new = 1023 - analogRead(PIN_REGULATOR);
#else
  uint16_t regulator_new = analogRead(PIN_REGULATOR);
#endif
  int32_t reg = regulator_new - regulator;
  if (reg < 0)
    reg = -reg;
  if (reg > DEBOUNCE_REGULATOR)
  {
    regulator = regulator_new;
    speed = map(regulator, 0, 1023, SPEED_A_MIN, SPEED_A_MAX);
    DDD("SPEED: ");
    DD(speed);
    return 1;
  }
  return 0;
}

void error()
{
#if PROTECT
  while (1)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(200);
  }
#endif
}
void setup()
{
  stepper.reverse(INVERT_DIR);
  stepper.disable();
  stepper.tick();
  // stepper.autoPower(true);

  btnA.setTickMode(AUTO);
  btnA.setDebounce(DEBOUNCE_A_B);

  btnB.setTickMode(AUTO);
  btnB.setDebounce(DEBOUNCE_A_B);

  f_onA = endA.state();
  f_onB = endB.state();
  speed = SPEED_A_MIN;
  pinMode(OUTPUT, LED_BUILTIN);
#ifdef DEBUG
  Serial.begin(9600);
#endif
  DD("START");
}

unsigned long timework = 0, time_auto_to_b = 0;
#ifdef DEBUG
unsigned long timer1 = 0;
#endif
void loop()
{
  if (btnA.state()) // Движение по кнопке А
  {
    f_onA = endA.state();
    f_onB = endB.state();
    DD("Кнопка A нажата");
    // f_manual = true;
    check_regulator(); // получаем скорость с крутилки
    stepper.enable();
    f_work = 1;
    stepper.setSpeed(-speed); // пишем скорость в двигло
    DD("Старт! К А");
    while (btnA.state()) // Пока кнопка зажата
    {
      if (endA.state()) // если концевик A зажат
      {
        DD("Конец А");
        f_onA = true;
        stepper.brake();
        stepper.tick();
        stepper.setCurrent(0);
      }
      else // тут нам разрешено ехать
      {
        if (endB.state() && !f_onB) // если сработал не тот концевик
        {
          DD("Неправильный конец B");
          f_onB = true;
          stepper.brake();
          stepper.disable();
          stepper.tick();
          error(); // Уходим в защиту и мигаем
        }

        if (f_onB && !endB.state())
        {
          DD("Отъехали от B");
          f_onB = false; // если мы отпустили концевик B
        }

        // едем едем в соседнее село...
        stepper.tick();
        if (check_regulator())
        {
          stepper.setSpeed(-speed);
        }
#ifdef DEBUG
        if (millis() - timer1 > 500)
        {
          timer1 = millis();
          DD(stepper.getCurrent());
        }
#endif
      }
    }
    timework = millis() + ADDWORK_TIME;
  }

  if (btnB.state()) // Движение по кнопке B
  {
    f_onA = endA.state();
    f_onB = endB.state();
    DD("Кнопка B нажата");
    // f_manual = true;
    // check_regulator(); // получаем скорость с крутилки
    stepper.enable();
    f_work = 1;
    stepper.setSpeed(SPEED_B); // пишем скорость в двигло
    DD("Старт! к B");
    while (btnB.state()) // Пока кнопка зажата
    {
      if (endB.state()) // если концевик B зажат
      {
        DD("Конец B");
        f_onB = true;
        stepper.brake();
        stepper.tick();
      }
      else // тут нам разрешено ехать
      {
        if (endA.state() && !f_onA) // если сработал не тот концевик
        {
          DD("Неправильный конец A");
          f_onA = true;
          stepper.brake();
          stepper.disable();
          stepper.tick();
          error(); // Уходим в защиту и мигаем
        }

        if (f_onA && !endA.state())
        {
          DD("Отъехали от A");
          f_onA = false; // если мы отпустили концевик A
        }

        // едем едем в соседнее село...
        stepper.tick();
        // if (check_regulator())
        // {
        //   stepper.setSpeed(speed);
        // }
#ifdef DEBUG
        if (millis() - timer1 > 500)
        {
          timer1 = millis();
          DD(stepper.getCurrent());
        }
#endif
      }
    }
    timework = millis() + ADDWORK_TIME;
  }

  if ((millis() > timework) && f_work)
  {
    DD("DISABLE!");
    stepper.disable();
    f_work = 0;
  }

  // TIMER_AUTO_TO_B
  if (endA.state()) // Движение от А к B
  {
    f_onA = endA.state();
    f_onB = endB.state();
    DD("A -> B START");
    stepper.enable();
    stepper.tick();
    delay(TIMER_AUTO_TO_B);
    f_work = 1;
    stepper.setSpeed(SPEED_B); // пишем скорость в двигло
    DD("Старт! A -> к Б");
    while (!endB.state()) // Пока не встретим B
    {
      // тут нам разрешено ехать
      if (endA.state() && !f_onA) // если сработал не тот концевик
      {
        DD("Неправильный конец A");
        f_onA = true;
        stepper.brake();
        stepper.disable();
        stepper.tick();
        error(); // Уходим в защиту и мигаем
      }

      if (f_onA && !endA.state())
      {
        DD("Отъехали от A");
        f_onA = false; // если мы отпустили концевик A
      }

      if (btnA.state() || btnB.state())
      {
        break;
      }
      // едем едем в соседнее село...
      stepper.tick();

#ifdef DEBUG
      if (millis() - timer1 > 500)
      {
        timer1 = millis();
        DD(stepper.getCurrent());
      }
#endif
    }
    if (endB.state())
    {
      DD("END_B AUTO");
      f_onB = 1;
      stepper.brake();
      stepper.tick();
    }
    timework = millis();
  }
  f_onA = endA.state();
  f_onB = endB.state();
}