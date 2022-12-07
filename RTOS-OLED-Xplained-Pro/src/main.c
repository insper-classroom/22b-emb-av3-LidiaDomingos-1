#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define BUZZER_PIO				PIOA
#define BUZZER_PIO_ID			ID_PIOA
#define BUZZER_PIO_IDX			21
#define BUZZER_PIO_IDX_MASK     (1 << BUZZER_PIO_IDX)

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

// LED da placa
// Acesso em n�vel baixo        (0)
// Apagado em n�vel baixo       (1)

#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Config do BUT1
#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)
// Config do BUT2
#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)
// Config do BUT3
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

// Definição dos LED's da placa OLED

// LED 1
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_PIO_IDX_MASK (1 << LED1_PIO_IDX)
// LED 2
#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_PIO_IDX_MASK (1 << LED2_PIO_IDX)
// LED 3
#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_PIO_IDX_MASK (1 << LED3_PIO_IDX)

/************************************************************************/
/* prototypes and types                                                 */
/************************************************************************/
int genius_get_sequence(int level, int *sequence);
void pin_toggle(Pio *pio, uint32_t mask);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) ;

/** prototypes */
void but_callback(void);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
static void io_init(void);

QueueHandle_t xQueueBtn;
/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}
static void io_init(void) {
	// Ativa PIO
	pmc_enable_periph_clk(LED_PIO_ID);
	
	// Configura como OUTPUT
	pio_configure(LED_PIO, PIO_OUTPUT_1, LED_IDX_MASK, PIO_DEFAULT);
	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP);

	// Ativa o PIO na qual o BUZZER foi conectado
	// para que possamos controlar o BUZZER.
	pmc_enable_periph_clk(BUZZER_PIO_ID);
	
	//Inicializa PA21 como sa?da
	pio_set_output(BUZZER_PIO, BUZZER_PIO_IDX_MASK, 0, 0, 0);
	// Configura led
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 1, 0, 0);

	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 1, 0, 0);

	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 1, 0, 0);

	// Como o botão do OLED é periférico.
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);


	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_PIO_PIN_MASK,
	PIO_IT_EDGE,
	but_callback);

	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but1_callback);

	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but2_callback);

	pio_handler_set(BUT3_PIO,
	BUT3_PIO_ID,
	BUT3_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but3_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_get_interrupt_status(BUT_PIO);

	// ativa interrupção
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);
	
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4
}

void but_callback(void) {
}
void but1_callback(void) {
	int k = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBtn, &k, &xHigherPriorityTaskWoken);
}
void but2_callback(void) {
	int k = 1;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBtn, &k, &xHigherPriorityTaskWoken);
}
void but3_callback(void) {
	int k = 2;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBtn, &k, &xHigherPriorityTaskWoken);
}

void TC0_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 0);

	// faz alguma coisa
	/** Muda o estado do LED (pisca) **/
	pin_toggle(BUZZER_PIO, BUZZER_PIO_IDX_MASK);
}

void TC3_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC1, 0);

	// faz alguma coisa
	/** Muda o estado do LED (pisca) **/
	pin_toggle(BUZZER_PIO, BUZZER_PIO_IDX_MASK);     
}

void TC6_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC2, 0);

	// faz alguma coisa
	/** Muda o estado do LED (pisca) **/
	pin_toggle(BUZZER_PIO, BUZZER_PIO_IDX_MASK);     
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
static void task_game(void *pvParameters) {
	gfx_mono_ssd1306_init();
	io_init();
	int nSequence = 0;
	int sequence[512];
	char strlevel[30];
	int level = 0;
	nSequence = genius_get_sequence(level, sequence);
	int d;
	int h = 0;
	int to_tocando = 1;
	
	TC_init(TC0, ID_TC0, 0, 1000);
	TC_init(TC1, ID_TC3, 0, 1500);
	TC_init(TC2, ID_TC6, 0, 2000);

	for (;;)  {
		while (to_tocando){
			h = 0;
			gfx_mono_draw_filled_rect(0,0, 128,32, GFX_PIXEL_CLR);
			sprintf(strlevel, "Level %d", level);
			gfx_mono_draw_string(strlevel, 20, 0, &sysfont);
			for (int i = 0; i < nSequence; i++){
				printf("%d\n",sequence[i]);
				if (sequence[i] == 0){
					pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
					tc_start(TC0, 0);
					delay_ms(500);
					pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
					delay_ms(500);
					tc_stop(TC0, 0);
				}
				if (sequence[i] == 1){
					pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
					tc_start(TC1, 0);
					delay_ms(500);
					pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
					delay_ms(500);
					tc_stop(TC1, 0);
				}
				if (sequence[i] == 2){
					pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
					tc_start(TC2, 0);
					delay_ms(500);
					pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
					delay_ms(500);
					tc_stop(TC2, 0);  
				}
				if (xQueueReceive(xQueueBtn, &d, 0)){
					gfx_mono_draw_filled_rect(0,0, 128,32, GFX_PIXEL_CLR);
					gfx_mono_draw_string("Fim de jogo", 20, 0, &sysfont);
					level = 0;
					nSequence = genius_get_sequence(level, sequence);
					delay_ms(2000);
					to_tocando = 1;
					break;
				}else{
					to_tocando = 0;
				}
			}

		}
		if (xQueueReceive(xQueueBtn, &d, ( TickType_t ) 800 )){
			if (d == 0){
				pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
				tc_start(TC0, 0);
				delay_ms(100);
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				delay_ms(100);
				tc_stop(TC0, 0);
			}
			if (d == 1){
				pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				tc_start(TC1, 0);
				delay_ms(100);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
				delay_ms(100);
				tc_stop(TC1, 0);  
			}
			if (d == 2){
				pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
				tc_start(TC2, 0);
				delay_ms(100);
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
				delay_ms(100);
				tc_stop(TC2, 0); 
			}
			if (d != sequence[h]){
				gfx_mono_draw_filled_rect(0,0, 128,32, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Fim de jogo", 20, 0, &sysfont);
				level = 0;
				nSequence = genius_get_sequence(level, sequence);
				to_tocando = 1;
				delay_ms(2000);
			}
			if(h == (nSequence - 1)){
				level = level + 1;
				nSequence = genius_get_sequence(level, sequence);
				to_tocando = 1;
				delay_ms(1000);
				gfx_mono_draw_filled_rect(0,0, 128,32, GFX_PIXEL_CLR);
				gfx_mono_draw_string("OK!", 20, 0, &sysfont);
				delay_ms(1000);
			}
			h++;
		}
		else{
			gfx_mono_draw_filled_rect(0,0, 128,32, GFX_PIXEL_CLR);
			gfx_mono_draw_string("Fim de jogo", 20, 0, &sysfont);
			level = 0;
			nSequence = genius_get_sequence(level, sequence);
			to_tocando = 1;
			delay_ms(2000);
		}
	}
}


/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
int genius_get_sequence(int level, int *sequence){
	int n = level + 3;

	for (int i=0; i< n ; i++) {
		*(sequence + i) = rand() % 3;
	}

	return n;
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_game, "game", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create game task\r\n");
	}

	xQueueBtn = xQueueCreate(100, sizeof(int));
	
	if (xQueueBtn == NULL){

		printf("falha em criar a queue \n");
	}
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
