/*
 * button.c
 *
 *  Created on: 21 Sep 2023
 *      Author: DwiSetyabudi
 */



#include "button.h"
#include "stdio.h"

uint32_t button_create(button_var* var, cyhal_gpio_t pin_t, uint8_t state, unsigned long duration_)
{
	var->pin_ = pin_t;
	var->State = state;
	if ( cyhal_gpio_init(var->pin_, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, !var->State) != CY_RSLT_SUCCESS )
		return 1;
	var->lastButtonState = var->lastState =!var->State; //lastState should be inverted from State
	var->duration = duration_;
	var->holdTime = 1000;
	var->DBInterval = 55;
	button_clearAllISR(var);
	return CY_RSLT_SUCCESS;
}

void button_SetHoldTime(button_var* var, unsigned long time_)
{
	var->holdTime = time_; // Set the hold time in seconds
}

void button_SetDebounceTime(button_var* var, unsigned long time_)
{
	var->DBInterval = time_;
}

void button_Pressed(button_var* var, void (*P)() )
{
	var->F_Pressed = P;
}

void delete_Button_pressed(button_var* var)
{
	var->F_Pressed = NULL;
}

void button_DoublePressed(button_var* var, void (*DP)() )
{
	  var->F_DPressed = DP;
}

void button_Held(button_var* var, void (*HOLD)() )
{
	var->F_Hold = HOLD;
}

void delete_Button_held(button_var* var)
{
	var->F_Hold = NULL;
}

void button_Released(button_var* var, void(*RELEASED_f)())
{
	var->F_Released = RELEASED_f;
}

void delete_Button_released(button_var* var)
{
	var->F_Released = NULL;
}

void delete_Button_doublePressed(button_var* var)
{
	var->F_DPressed = NULL;
}

void button_clearAllISR(button_var* var)
{
	var->F_Pressed  = NULL;
	var->F_DPressed = NULL;
	var->F_Hold     = NULL;
	var->F_Released = NULL;
}

uint8_t button_check(button_var* var)
{
  var->output = 0;
  uint8_t button_ = cyhal_gpio_read(var->pin_);
  if ( button_ != var->lastButtonState)
  {
	  var->lastDebounceTime = button.tick();
	  var->lastButtonState = button_;
	  var->time = button.tick();
  }


  // Check for Rollover
  unsigned long RO_Time = button.tick(); // current time into RollOver variable
  if (RO_Time < var->time) // is the RollOver variable smaller than ontime?
	var->time = RO_Time; // if yes,  reset ontime to zero

  while ( (button.tick() - var->time) <= var->duration)
  {
	 // button = digitalRead(ButtonPin);     // read the button
	 if (button_ != var->lastState) // see if the button is not held down.
	 {
	   if (button_ == !var->State) // button was released
	   {
		 if ((button.tick() - var->lastDebounceTime) >= var->DBInterval) // button debounce, very important
		 {
		   var->output++;                    // increment a counter, but only when the button is pressed then released
		   var->lastDebounceTime = button.tick(); // update the debounce time
		 }
	   }
	   var->lastState = cyhal_gpio_read(var->pin_); // update the buttons last state with it's new state
	 }
	 button_ = cyhal_gpio_read(var->pin_);
  }
  if(button_ == var->State && button_ == var->lastButtonState)
	if( (var->HeldTime = (button.tick() - var->time)) > var->holdTime )
		var->output = HELD_btn;

  switch (var->output)
  {
	case WAITING_btn:
	  {
		if( var->lastOut != WAITING_btn &&  var->lastOut != RELEASED_btn)
		{
		  if( var->F_Released != NULL )
			  var->F_Released();
		  var->output  = RELEASED_btn;
		  var->lastOut = WAITING_btn;
		}
		else
		{
			var->lastOut = var->output = WAITING_btn;
			var->output  = WAITING_btn;
		}

	  }
	  break;

	case PRESSED_btn:
	  if (var->F_Pressed!=NULL)
		  var->F_Pressed();
	  break;

	case DOUBLE_PRESSED_btn:
	  if (var->F_DPressed!=NULL)
		  var->F_DPressed();
	  break;
	case HELD_btn:
	  if (var->F_Hold!=NULL)
		  var->F_Hold();
	  break;
  }
  var->lastOut = var->output;
  return var->output; // return the output count
}

float button_GetHeldTime(button_var* var, float divisor)
{
  if (divisor > 0)
	return var->HeldTime / divisor;
  else
	return -1;
}

static void button_setTick( uint32_t (*tick)())
{
	button.tick = tick;
}

buttonF button = {
		button_create,
		button_SetHoldTime,
		button_SetDebounceTime,

		button_Pressed,
		button_DoublePressed,
		button_Held,
		button_Released,

		delete_Button_held,
		delete_Button_pressed,
		delete_Button_released,
		delete_Button_doublePressed,
		button_clearAllISR,

		button_check,
		button_GetHeldTime,
		button_setTick,
		.tick = NULL,
};
