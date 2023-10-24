#pragma once

#include  <PowerOut.h>

namespace Outputs
{
	/* Настройки */
	static constexpr uint8_t CFG_PortCount = 6;			// Кол-во портов управления.
	static constexpr uint32_t CFG_RefVoltage = 3337000;	// Опорное напряжение, микровольты.
	static constexpr uint8_t CFG_INA180_Gain = 20;		// Усиление микросхемы INA180.
	static constexpr uint8_t CFG_ShuntResistance = 5;	// Сопротивление шунта, миллиомы.
	/* */
	
	PowerOut<CFG_PortCount> outObj(CFG_RefVoltage, CFG_INA180_Gain, CFG_ShuntResistance);
	
	void OnShortCircuit(uint8_t num, uint16_t current)
	{

	}
	
	inline void Setup()
	{
		outObj.AddPort( {GPIOB, GPIO_PIN_11}, {GPIOA, ADC_CHANNEL_6}, 10000 );
		outObj.AddPort( {GPIOB, GPIO_PIN_10}, {GPIOA, ADC_CHANNEL_5}, 5000 );
		outObj.AddPort( {GPIOB, GPIO_PIN_2}, {GPIOA, ADC_CHANNEL_4}, 10000 );
		outObj.AddPort( {GPIOB, GPIO_PIN_1}, {GPIOA, ADC_CHANNEL_3}, 10000 );
		outObj.AddPort( {GPIOB, GPIO_PIN_0}, {GPIOA, ADC_CHANNEL_2}, 5000 );
		outObj.AddPort( {GPIOA, GPIO_PIN_7}, {GPIOA, ADC_CHANNEL_1}, 5000 );
		outObj.Init();

		//outObj.On(4);
		//outObj.On(6);
		//outObj.Off(1);
		outObj.RegShortCircuitEvent(OnShortCircuit);
		//outObj.Current(1);

		//outObj.SetOn(6, 250, 500);
		//outObj.SetOn(5, 1000, 100);
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		outObj.Processing(current_time);
		
		static uint32_t last_time = 0;
		if(current_time - last_time > 250)
		{
			last_time = current_time;
			
			for(uint8_t i = 1; i < CFG_PortCount+1; ++i)
			{
				Logger.PrintTopic("POUT").Printf("Port: %d, current: %5d;", i, outObj.GetCurrent(i)).PrintNewLine();
			}
		}
		
		current_time = HAL_GetTick();
		
		return;
	}
}
