#pragma once

//#define MATRIX_STEP_BY_STEP
#include <MatrixLed.h>

extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch1;

namespace Matrix
{

	/* Настройки */
	static constexpr uint8_t CFG_Layers = 8;	// Кол-во слоёв анимации.
	static constexpr uint8_t CFG_Width = 128;	// Ширина экрана.
	static constexpr uint8_t CFG_Height = 16;	// Высота экрана.
	static constexpr uint16_t CFG_Delay = 200;	// Интервал обновления экрана.
	#define ROOT_DIRECTORY ("/led_pxl_f")		// Папка с файлами pxl.
	/* */
	
	MatrixLed<CFG_Layers, CFG_Width, CFG_Height> matrixObj(CFG_Delay);
	
	uint8_t *frame_buffer_ptr;
	uint16_t frame_buffer_len;
	volatile uint16_t frame_buffer_idx = 0;
	volatile uint8_t dma_buffer[ (8 * 3 * 4) ];		// 8 бит * 3 цвета * 4 пикселя.
	
	volatile uint8_t PWM_HI;	// PWM Code HI Log.1 period
	volatile uint8_t PWM_LO;	// PWM Code LO Log.1 period




#define WS2812       ///< Family: {WS2811S, WS2811F, WS2812, SK6812}
// WS2811S — RGB, 400kHz;
// WS2811F — RGB, 800kHz;
// WS2812  — GRB, 800kHz;
// SK6812  — RGBW, 800kHz

#define TIM_NUM	   2  ///< Timer number
#define TIM_CH	   TIM_CHANNEL_1  ///< Timer's PWM channel
#define DMA_HANDLE hdma_tim2_ch1  ///< DMA Channel

/// Timer handler
#if TIM_NUM == 1
#define TIM_HANDLE  htim1
#elif TIM_NUM == 2
#define TIM_HANDLE  htim2
#elif TIM_NUM == 3
#define TIM_HANDLE  htim3
#elif TIM_NUM == 4
#define TIM_HANDLE  htim4
#elif TIM_NUM == 5
#define TIM_HANDLE  htim5
#elif TIM_NUM == 8
#define TIM_HANDLE  htim8
#else
#error Wrong timer! Fix it in ARGB.h string 41
#warning If you shure, set TIM_HANDLE and APB ring by yourself
#endif

/// Timer's RCC Bus
#if TIM_NUM == 1 || (TIM_NUM >= 8 && TIM_NUM <= 11)
#define APB1
#else
#define APB2
#endif



void DMAInit()
{
    /* Auto-calculation! */
    uint32_t APBfq; // Clock freq
#ifdef APB1
    APBfq = HAL_RCC_GetPCLK1Freq();
    APBfq *= (RCC->CFGR & RCC_CFGR_PPRE1) == 0 ? 1 : 2;
#endif
#ifdef APB2
    APBfq = HAL_RCC_GetPCLK2Freq();
    APBfq *= (RCC->CFGR & RCC_CFGR_PPRE2) == 0 ? 1 : 2;
#endif
#ifdef WS2811S
    APBfq /= (uint32_t) (400 * 1000);  // 400 KHz - 2.5us
#else
    APBfq /= (uint32_t) (800 * 1000);  // 800 KHz - 1.25us
#endif
    TIM_HANDLE.Instance->PSC = 0;                        // dummy hardcode now
    TIM_HANDLE.Instance->ARR = (uint16_t) (APBfq - 1);   // set timer prescaler
    TIM_HANDLE.Instance->EGR = 1;                        // update timer registers
#if defined(WS2811F) || defined(WS2811S)
    PWM_HI = (uint8_t) (APBfq * 0.48) - 1;     // Log.1 - 48% - 0.60us/1.2us
    PWM_LO = (uint8_t) (APBfq * 0.20) - 1;     // Log.0 - 20% - 0.25us/0.5us
#endif
#ifdef WS2812
    PWM_HI = (uint8_t) (APBfq * 0.56) - 1;     // Log.1 - 56% - 0.70us
    PWM_LO = (uint8_t) (APBfq * 0.28) - 1;     // Log.0 - 28% - 0.35us
#endif
#ifdef SK6812
    PWM_HI = (uint8_t) (APBfq * 0.48) - 1;     // Log.1 - 48% - 0.60us
    PWM_LO = (uint8_t) (APBfq * 0.24) - 1;     // Log.0 - 24% - 0.30us
#endif

    TIM_CCxChannelCmd(TIM_HANDLE.Instance, TIM_CH, TIM_CCx_ENABLE); // Enable GPIO to IDLE state
//		ARGB_LOC_ST = ARGB_READY; // Set Ready Flag
    HAL_Delay(1); // Make some delay
		
}

#if TIM_CH == TIM_CHANNEL_1
#define ARGB_TIM_DMA_ID TIM_DMA_ID_CC1
#define ARGB_TIM_DMA_CC TIM_DMA_CC1
#define ARGB_TIM_CCR CCR1
#elif TIM_CH == TIM_CHANNEL_2
#define ARGB_TIM_DMA_ID TIM_DMA_ID_CC2
#define ARGB_TIM_DMA_CC TIM_DMA_CC2
#define ARGB_TIM_CCR CCR2
#elif TIM_CH == TIM_CHANNEL_3
#define ARGB_TIM_DMA_ID TIM_DMA_ID_CC3
#define ARGB_TIM_DMA_CC TIM_DMA_CC3
#define ARGB_TIM_CCR CCR3
#elif TIM_CH == TIM_CHANNEL_4
#define ARGB_TIM_DMA_ID TIM_DMA_ID_CC4
#define ARGB_TIM_DMA_CC TIM_DMA_CC4
#define ARGB_TIM_CCR CCR4
#endif





static void RGB_TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
static void RGB_TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);








void DMADraw()
{

	//frame_buffer_idx = 0;

    //ARGB_LOC_ST = ARGB_BUSY;
    if (frame_buffer_idx != 0 || DMA_HANDLE.State != HAL_DMA_STATE_READY) {
        return;
    } 
		else {
        for (volatile uint8_t i = 0; i < 8; i++) {
            // set first transfer from first values
            dma_buffer[i] = (((frame_buffer_ptr[0] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 8] = (((frame_buffer_ptr[1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 16] = (((frame_buffer_ptr[2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 24] = (((frame_buffer_ptr[3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 32] = (((frame_buffer_ptr[4] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 40] = (((frame_buffer_ptr[5] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
					
			dma_buffer[i + 48] = (((frame_buffer_ptr[6] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 56] = (((frame_buffer_ptr[7] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 64] = (((frame_buffer_ptr[8] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
			dma_buffer[i + 72] = (((frame_buffer_ptr[9] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 80] = (((frame_buffer_ptr[10] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 88] = (((frame_buffer_ptr[11] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
        }
        HAL_StatusTypeDef DMA_Send_Stat = HAL_ERROR;
        while (DMA_Send_Stat != HAL_OK) {
            if (TIM_CHANNEL_STATE_GET(&TIM_HANDLE, TIM_CH) == HAL_TIM_CHANNEL_STATE_BUSY) {
                DMA_Send_Stat = HAL_BUSY;
                continue;
            } else if (TIM_CHANNEL_STATE_GET(&TIM_HANDLE, TIM_CH) == HAL_TIM_CHANNEL_STATE_READY) {
                TIM_CHANNEL_STATE_SET(&TIM_HANDLE, TIM_CH, HAL_TIM_CHANNEL_STATE_BUSY);
            } else {
                DMA_Send_Stat = HAL_ERROR;
                continue;
            }
						// Callback
            TIM_HANDLE.hdma[ARGB_TIM_DMA_ID]->XferCpltCallback = RGB_TIM_DMADelayPulseCplt;
            TIM_HANDLE.hdma[ARGB_TIM_DMA_ID]->XferHalfCpltCallback = RGB_TIM_DMADelayPulseHalfCplt;
            TIM_HANDLE.hdma[ARGB_TIM_DMA_ID]->XferErrorCallback = TIM_DMAError;
						// DMA init
            if (HAL_DMA_Start_IT(TIM_HANDLE.hdma[ARGB_TIM_DMA_ID], (uint32_t) dma_buffer,
                                 (uint32_t) &TIM_HANDLE.Instance->ARGB_TIM_CCR,
                                 (uint32_t) sizeof(dma_buffer)) != HAL_OK) {
                DMA_Send_Stat = HAL_ERROR;
                continue;
            }
            __HAL_TIM_ENABLE_DMA(&TIM_HANDLE, ARGB_TIM_DMA_CC);
            if (IS_TIM_BREAK_INSTANCE(TIM_HANDLE.Instance) != RESET)
                __HAL_TIM_MOE_ENABLE(&TIM_HANDLE);
            if (IS_TIM_SLAVE_INSTANCE(TIM_HANDLE.Instance)) {
                uint32_t tmpsmcr = TIM_HANDLE.Instance->SMCR & TIM_SMCR_SMS;
                if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
                    __HAL_TIM_ENABLE(&TIM_HANDLE);
            } else
                __HAL_TIM_ENABLE(&TIM_HANDLE);
            DMA_Send_Stat = HAL_OK;
        }
        frame_buffer_idx = 12;
        return;
    }
}

static void RGB_TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma) {

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *) ((DMA_HandleTypeDef *) hdma)->Parent;
    // if wrong handlers
    if (hdma != &DMA_HANDLE || htim != &TIM_HANDLE) return;
    if (frame_buffer_idx == 0) return; // if no data to transmit - return
    // if data transfer
    if (frame_buffer_idx < frame_buffer_len) {
        // fill first part of buffer
			

        for (volatile uint8_t i = 0; i < 8; i++) {
					
            dma_buffer[i] = (((frame_buffer_ptr[frame_buffer_idx + 0] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 8] = (((frame_buffer_ptr[frame_buffer_idx + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 16] = (((frame_buffer_ptr[frame_buffer_idx + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
					
            dma_buffer[i + 24] = (((frame_buffer_ptr[frame_buffer_idx + 3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 32] = (((frame_buffer_ptr[frame_buffer_idx + 4] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 40] = (((frame_buffer_ptr[frame_buffer_idx + 5] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;		
					
        }
        frame_buffer_idx += 6;			// old set ++
    } else if (frame_buffer_idx < frame_buffer_len + 12) { // if RET transfer
        memset((uint8_t *) &dma_buffer[0], 0, (sizeof(dma_buffer) / 2)); // first part
        frame_buffer_idx += 6;			// old set ++
    }
}

static void RGB_TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma) {
	
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *) ((DMA_HandleTypeDef *) hdma)->Parent;
    // if wrong handlers
    if (hdma != &DMA_HANDLE || htim != &TIM_HANDLE) return;
    if (frame_buffer_idx == 0) return; // if no data to transmit - return
	

    if (hdma == htim->hdma[TIM_DMA_ID_CC1]) {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;
        if (hdma->Init.Mode == DMA_NORMAL) {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_1, HAL_TIM_CHANNEL_STATE_READY);
        }
    } else if (hdma == htim->hdma[TIM_DMA_ID_CC2]) {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;
        if (hdma->Init.Mode == DMA_NORMAL) {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_READY);
        }
    } else if (hdma == htim->hdma[TIM_DMA_ID_CC3]) {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;
        if (hdma->Init.Mode == DMA_NORMAL) {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_3, HAL_TIM_CHANNEL_STATE_READY);
        }
    } else if (hdma == htim->hdma[TIM_DMA_ID_CC4]) {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;
        if (hdma->Init.Mode == DMA_NORMAL) {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_4, HAL_TIM_CHANNEL_STATE_READY);
        }
    } else {
        //nothing to do 
    }


// if data transfer
    if (frame_buffer_idx < frame_buffer_len) {
        // fill second part of buffer
			

        for (volatile uint8_t i = 0; i < 8; i++) {
					
            dma_buffer[i + 48] = (((frame_buffer_ptr[frame_buffer_idx + 0] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 56] = (((frame_buffer_ptr[frame_buffer_idx + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 64] = (((frame_buffer_ptr[frame_buffer_idx + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
					
            dma_buffer[i + 72] = (((frame_buffer_ptr[frame_buffer_idx + 3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 80] = (((frame_buffer_ptr[frame_buffer_idx + 4] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            dma_buffer[i + 88] = (((frame_buffer_ptr[frame_buffer_idx + 5] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
					
					
        }
        frame_buffer_idx += 6;				// old set ++
    } else if (frame_buffer_idx < frame_buffer_len + 12) { // if RET transfer
        memset((uint8_t *) &dma_buffer[sizeof(dma_buffer) / 2], 0, (sizeof(dma_buffer) / 2)); // second part
        frame_buffer_idx += 6;				// old set ++
    } else { // if END of transfer
			
        frame_buffer_idx = 0;
        // STOP DMA:


        __HAL_TIM_DISABLE_DMA(htim, ARGB_TIM_DMA_CC);
        (void) HAL_DMA_Abort_IT(htim->hdma[ARGB_TIM_DMA_ID]);


        if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET) {
            /* Disable the Main Output */
            __HAL_TIM_MOE_DISABLE(htim);
        }
        /* Disable the Peripheral */
        __HAL_TIM_DISABLE(htim);
        /* Set the TIM channel state */
        TIM_CHANNEL_STATE_SET(htim, TIM_CH, HAL_TIM_CHANNEL_STATE_READY);
        //ARGB_LOC_ST = ARGB_READY;
				
    }
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}








inline void Setup()
{

#ifdef ROOT_DIRECTORY
	f_chdir(ROOT_DIRECTORY);
#endif
	
	matrixObj.RegLayer("layer0.pxl", 0);	// 0 - Фон / Заливка;
	matrixObj.RegLayer("layer1.pxl", 1);	// 1 - Анимация;
	matrixObj.RegLayer("layer2.pxl", 2);	// 2 - Габариты;
	matrixObj.RegLayer("layer3.pxl", 3);	// 3 - Задних ход;
	matrixObj.RegLayer("layer4.pxl", 4);	// 4 - Стопы;
	matrixObj.RegLayer("layer5.pxl", 5);	// 5 - Повтороты лево;
	matrixObj.RegLayer("layer6.pxl", 6);	// 6 - Повтороты право;
	matrixObj.RegLayer("layer7.pxl", 7);	// 7 - Аварийка;

	matrixObj.ShowLayer(0);
	matrixObj.ShowLayer(1);
	//matrixObj.ShowLayer(2);
	//matrixObj.ShowLayer(3);
	//matrixObj.ShowLayer(4);
	//matrixObj.ShowLayer(5);
	//matrixObj.ShowLayer(6);
	//matrixObj.ShowLayer(7);
	
	matrixObj.SetBrightness(10);
	
	matrixObj.GetFrameBuffer(frame_buffer_ptr, frame_buffer_len);

	//matrixObj.ManualMode(true);
	//matrixObj.DrawPixel(5, 0xFF0000FF);
	//matrixObj.DrawPixel(6, 0xFF00FF00);
	//matrixObj.DrawPixel(7, 0xFFFF0000);
	//matrixObj.DrawPixel(8, 0xAAFFFFFF);
	//matrixObj.DrawPixel(0, 0, 0xFF0000FF);
	//matrixObj.DrawPixel(5, 2, 0xFF00FF00);
	//matrixObj.DrawPixel(6, 3, 0xFFFF0000);
	//matrixObj.ManualDraw();
	
	DMAInit();
	
	return;
}

uint32_t timer1, timer2, timer3, timer4;

inline void Loop(uint32_t &current_time)
{
	timer1 = HAL_GetTick();
	matrixObj.Processing(current_time);
	timer2 = HAL_GetTick();
	
	if(matrixObj.IsBufferReady() == true)
	{
		matrixObj.SetFrameDrawStart();
		
		DMADraw();

		//Serial::Printf("+INFO\tRenderTime: %d ms;\r\n", (timer2 - timer1));
		Logger.PrintTopic("INFO").Printf("RenderTime: %d ms;", (timer2 - timer1)).PrintNewLine();
		
		//Serial::Print("+PXL=128,16,2\r\n");
		//Serial::Print(frame_buffer_ptr, frame_buffer_len);
	}
	
	if( matrixObj.GetFrameIsDraw() == true && frame_buffer_idx == 0 )
	{
		matrixObj.SetFrameDrawEnd();
	}
	
	current_time = HAL_GetTick();
	
	return;
}

}
