#include "stm32f10x.h"
#include "core_cm3.h"
#include "arm_etm.h"
#include "sys.h"
#include "timer.h"
#include "led.h"
//#include "usart.h"
#include "stdio.h"

//uint32_t *ETM_Lock_Access =           (uint32_t*)0xE0041FB0;
//uint32_t *ETM_Control =               (uint32_t*)0xE0041000;
//uint32_t *ETM_Status =                (uint32_t*)0xE0041010;
//uint32_t *ETM_Trigger_Event =         (uint32_t*)0xE0041008;
//uint32_t *ETM_Trace_Enable_Control =  (uint32_t*)0xE004101C;
//uint32_t *ETM_Trace_Enable_Event =    (uint32_t*)0xE0041020;
//uint32_t *ETM_Trace_Start_Stop =      (uint32_t*)0xE0041024;


int globalCounter = 0; // For watchpoint

void hardfault_handler(void) { for(;;); }

void bubble_sort (int *a, int n);
void configure_tracing()
{
    /* STM32 specific configuration to enable the TRACESWO IO pin */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= (2 << 24); // Disable JTAG to release TRACESWO
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins
    
    if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    {
        // Some (all?) STM32s don't allow writes to DBGMCU register until
        // C_DEBUGEN in CoreDebug->DHCSR is set. This cannot be set by the
        // CPU itself, so in practice you need to connect to the CPU with
        // a debugger once before resetting it.
        return;
    }
    
    /* Configure Trace Port Interface Unit */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to registers
    TPI->ACPR = 8; // Trace clock = HCLK/(x+1) = 8MHz    这里HCLK是处理器时钟,这个值的初始值是0，0+1=1就是没有分频,还注释说是8MHz那是因为F105是8MHz
    TPI->SPPR = 2; // Pin protocol = NRZ/USART
    
    TPI->FFCR = 0x102;
    //TPI->FFCR = 0x100; //为什么设置102开启ETM后，ITM输出就不对了？？,在博客http://essentialscrap.com/tips/arm_trace/theory.html 中说到启用ETM会生成大量数据将使ITM追踪事件的数据丢失
    		       // TPIU packet framing enabled when bit 2 is set.
    		       // You can use 0x102 if you need both DWT/ITM and ETM.
                       // You can use 0x100 if you only need DWT/ITM and not ETM.
   
    /* Configure PC sampling and exception trace  */
    DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) // Prescaler for PC sampling
                                           // 0 = x32, 1 = x512  //
                    //bits[6]就是DWT_CYCCNT寄存器的0到5bit，那么节拍就是32记录一次PC,bits[10]就是0到9bit，那么节拍记录就是512记录一次PC
              | (0 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
                                                // Divider = value + 1
              | (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
              | (2 << DWT_CTRL_SYNCTAP_Pos)    // Sync packet interval
                                               // 0 = Off, 1 = Every 2^23 cycles,
                                               // 2 = Every 2^25, 3 = Every 2^27
              | (1 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
              | (1 << DWT_CTRL_CYCCNTENA_Pos); // Enable cycle counter
    
    /* Configure instrumentation trace macroblock */
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos) // Trace bus ID for TPIU
             | (1 << ITM_TCR_DWTENA_Pos) // Enable events from DWT
             | (1 << ITM_TCR_SYNCENA_Pos) // Enable sync packets
             | (1 << ITM_TCR_ITMENA_Pos); // Main enable for ITM
    ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports


    //ETM_Lock_Access = 0xC5ACCE55;
    //ETM_Control = 0x00001D1E;
    //ETM_Trigger_Event = 0x0000406F;
    //ETM_Trace_Enable_Event = 0x0000006F;
    //ETM_Trace_Start_Stop = 0x00000001;


   /* Configure embedded trace macroblock */
    ETM->LAR = 0xC5ACCE55;
    ETM_SetupMode();
    ETM->CR = ETM_CR_ETMEN // Enable ETM output port
            | ETM_CR_STALL_PROCESSOR // Stall processor when fifo is full
            | ETM_CR_BRANCH_OUTPUT // Report all branches
            //| (1 << 4);//port_size位21,6,5,4是0001表示8bit，这里目前认为复位值都是0,那么把第4位改为1就行
            | ETM_CR_PORTSIZE_8BIT;
    ETM->TRACEIDR = 2; // Trace bus ID for TPIU
    ETM->TECR1 = ETM_TECR1_EXCLUDE; // Trace always enabled
    ETM->FFRR = ETM_FFRR_EXCLUDE; // Stalling always enabled
    ETM->FFLR = 24; // Stall when less than N bytes free in FIFO (range 1..24)
                    // Larger values mean less latency in trace, but more stalls.
    // Note: we do not enable ETM trace yet, only for specific parts of code.
    
    ETM->TRIGGER = 0x0000406F;
    ETM->TEEVR = 0x0000006F;
    //ETM->TSSCR = 0x00000001;

}

void configure_watchpoint()
{
    /* This is an example of how to configure DWT to monitor a watchpoint.
       The data value is reported when the watchpoint is hit. */
    
    /* Monitor all accesses to GPIOC (range length 32 bytes) */
    //DWT->COMP0 = (uint32_t)bubble_sort;                      //改为了比GPIOC
//    DWT->COMP0 = (uint32_t)GPIOC;                      //改为了比较GPIOC
//    DWT->MASK0 = 8;							 //屏蔽掉数据地址的后5位，目前DWT->COMP0的值是GPIOA的地址:0x40010800
//											//可能是出于加快比较速度的原因吧，那为什么不把MASK[3:0]
//										        //设置为8,反正0x40010800最后八位都是0
//										        
//    DWT->FUNCTION0 = (2 << DWT_FUNCTION_FUNCTION_Pos) // Report data and addr on watchpoint hit
//                   | (1 << DWT_FUNCTION_EMITRANGE_Pos);
//                   //| (1 << DWT_FUNCTION_CYCMATCH_Pos);
    
    /* Monitor all accesses to globalCounter (range length 4 bytes) */
    DWT->COMP1 = (uint32_t)&globalCounter;
    DWT->MASK1 = 2;
    DWT->FUNCTION1 = //(3 << DWT_FUNCTION_FUNCTION_Pos); // Report data and PC on watchpoint hit
                     (2 << DWT_FUNCTION_FUNCTION_Pos);
}

void ITM_Print(int port, const char *p)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port)))
    {
        while (*p)
        {
            while (ITM->PORT[port].u32 == 0);
            ITM->PORT[port].u8 = *p++;
        }   
    }
}

void ITM_SendValue (int port, uint32_t value)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port)))
    {
        while (ITM->PORT[port].u32 == 0);
        ITM->PORT[port].u32 = value;
    }
}

void delay(uint32_t n)
{
    int i;
    for (i = 0; i < n; i++);//10000个空指令大约是0.138ms
    {asm("nop");}
}

void bubble_sort (int *a, int n) 
{
    //globalCounter = 0x0D;
    int i, t, s = 1;
    while (s) 
    {
        s = 0;
        for (i = 1; i < n; i++) 
        {
            if (a[i] < a[i - 1]) 
            {
                t = a[i];
                a[i] = a[i - 1];
                a[i - 1] = t;
                s = 1;            
            }
        }
    }
}


int main(void)
{
    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;
    int values[5] = {35,2,235,11,2};
    uint32_t r;
    uint32_t r_address;

    configure_tracing();
    configure_watchpoint();

    //GPIOC->CRL &= 0xFFF00FFF;
    //GPIOC->CRL |= 0x00033000;//设置PC3，PC4为输出

    LED_Init();

    TIM3_Int_Init(50,7199);//10KHz的频率，计数到5000为500ms

    //uart_init(115200);


    while(1)
    {

        a++;
        b = a + 1;
        //LED0 = 0;
        LED1 = 0;
        delay(10);
        //LED1 = 0;
        LED1 = 1;
        //printf("LED1");
        //ITM_Print(0,"LED1");

        
        ETM_TraceMode();
        bubble_sort(values, 5);
        ETM_SetupMode();

        if(a == 10)
        {
            //globalCounter = 0x10;
        }
      
        if(a == 15)
        {
            //ETM_TraceMode();
            //GPIOC->BSRR = (1 << 4);
      
           //ITM_Print(0, "first");
            bubble_sort(values, 5);
            
            //GPIOC->BRR = (1 << 4);
            //ETM_SetupMode();
        }
      
        bubble_sort(values, 5);
        if(a == 20)
        {
            //ITM_Print(0,"second");
            //globalCounter = 0x20;
            a = 0;
        }
        r = ETM->CCR;
        r_address = &(ETM->CCR);
        
        delay(100);
        globalCounter = 0x0A;
        delay(100);
        globalCounter = 0x0B;
        delay(100);
        globalCounter = 0x0C;
        delay(100);
        globalCounter = 0x0D;
        delay(100);
        globalCounter = 0x0E;
        delay(100);
        globalCounter = 0x0F;
        delay(100);
        globalCounter = 0x10;
        delay(100);
        globalCounter = 0x11;
        delay(100);
        globalCounter = 0x12;
        delay(100);
        globalCounter = 0x13;

        delay(100);
    }
}

//void* myvectors[] 
//__attribute__ ((section("vectors")))= {
//    (void*)0x20002000, // Stack ptr
//    main,       // Reset addr
//    hardfault_handler,
//    hardfault_handler,
//    [16 + TIM3_IRQn] = TIM3_IRQHandler
//};

