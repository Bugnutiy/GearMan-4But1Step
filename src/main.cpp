#include <Arduino.h>
#include "GyverButton.h"
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
#define PIN_STEP 4    // Пин STEP
#define PIN_DIR 5     // Пин dir
#define PIN_ENABLE 13 // Пин enable, возможно он называется set

// Пины кнопок управления
#define PIN_BUTTON_A 7 // Пин кнопки движения к концевику A
#define PIN_BUTTON_B 8 // Пин кнопки движения к концевику B

// Пины концевиков
#define PIN_END_A 9  // Пин концевика A
#define PIN_END_B 10 // Пин концевика B

// Потенциометр (Крутилка)
#define PIN_REGULATOR A0 // Пин регулятора скорости

/**************************Настройки**************************/
// Скорость
#define SPEED_A_MIN 80  // Минимальная скорость в сторону А
#define SPEED_A_MAX 350 // Максимальная скорость в сторону A
#define SPEED_B 150     // Постоянная скорость в сторону B

// Двигатель
#define ENABLE_SIGNAL LOW // Какой сигнал enable разрешает движение: HIGH (+5v) или LOW (GND)
#define INVERT_DIR false  // если мотор крутится не туда, то меняем значение true/false
#define GS_NO_ACCEL       // Эта строчка убирает плавный старт и плавный тормоз моторов, делая их резкими

// Кнопки управления
#define DEBOUNCE_A_B 50 // Устранить дребезг кнопок A и B (больше = медленнее и точнее)
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

// Потенциометр
#define DEBOUNCE_REGULATOR 1 // чем больше, тем стабильнее работает крутилка

/****************************КОД*****************************/
#include "GyverStepper2.h"
#ifdef DEBUG
#define DD(x) Serial.println(x)
#else
#define DD(x)
#endif
GStepper2<STEPPER2WIRE> stepper(0, PIN_STEP, PIN_DIR, PIN_ENABLE); // Шаговый двигатель
void setup()
{
  stepper.reverse(INVERT_DIR);
  stepper.autoPower(true);

  btnA.setTickMode(AUTO);
  btnA.setDebounce(DEBOUNCE_A_B);

  btnB.setTickMode(AUTO);
  btnB.setDebounce(DEBOUNCE_A_B);
#ifdef DEBUG
  Serial.println(9600);
#endif
  DD("START");
}

#define END_A_TRUE END_A_CONNECTION ^ END_A_TYPE
#define END_B_TRUE END_B_CONNECTION ^ END_B_TYPE
#if END_A_CONNECTION
#define END_A_PINMODE INPUT
#else
#define END_A_PINMODE INPUT_PULLUP
#endif
class endBtn
{
private:
  /* data */
public:
  endBtn(byte pin, byte con_type, byte truth);
  ~endBtn();
};

endBtn::endBtn(byte pin, byte con_type, byte typ)
{
}
endBtn::~endBtn()
{
}

bool f_manual;
void loop()
{
  if (btnA.isPress())
  {
    f_manual = true;
    while (btnA.state())
    {
    }
  }
}