#include <stdio.h>             // Biblioteca padrão para entrada e saída, utilizada para printf.
#include "pico/stdlib.h"       // Biblioteca padrão para funções básicas do Pico, como GPIO e temporização.
#include "hardware/adc.h"      // Biblioteca para controle do ADC (Conversor Analógico-Digital).
#include "hardware/pwm.h"      // Biblioteca para controle de PWM.
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/clocks.h"
#include "pico/stdio_usb.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ENDERECO 0x3C

#define WIDTH 128
#define HEIGHT 64

// Definições dos pinos para o joystick, botões e LEDs
#define VRY_PIN 27    // Define o pino GP26 para o eixo X do joystick (Canal ADC0)
#define VRX_PIN 26    // Define o pino GP27 para o eixo Y do joystick (Canal ADC1)
#define BOTAO_JOYSTICK 22     // Define o pino GP22 para o botão do joystick (entrada digital)
#define LED_VERMELHO 13  // Define o pino GP13 para o LED vermelho (PWM)
#define LED_AZUL 12  // Define o pino GP12 para o LED azul (PWM)
#define LED_VERDE 11 // Define o pino GP11 para o LED verde (PWM)
#define BOTAO_A 5   // Define o pino GP5 para o botão A da BitDogLab

// Função para mapear valores
#define MAP_x(value, in_min, in_max, out_min, out_max) \
    (59-((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min))

#define MAP_y(value, in_min, in_max, out_min, out_max) \
    ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

uint duty_cycle_led_azul;
uint duty_cycle_led_vermelho;
uint vrx_value;
uint vry_value;
uint x_value;
uint y_value;
uint pos_x = 3;
uint pos_y = 3;
static uint8_t last_pos_x = 0;
static uint8_t last_pos_y = 0;
static volatile bool flag_display = false;
uint border_style = 0; // Estilo da borda: 0 = borda simples, 1 = borda tracejada

ssd1306_t ssd; // Variável global para o display

// Parâmetros para debounce
#define DEBOUNCE_DELAY_MS 50

// Variáveis para armazenar o estado dos botões
bool button_a_state = false;

bool estado_led_verde = false;
bool estado_led_azul = true;
bool estado_led_vermelho = true;

// Função para verificar o debounce de um botão
bool debounce(uint pin, bool *last_state) {
    static uint32_t last_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    bool current_state = gpio_get(pin) == 0; // 0 indica que o botão está pressionado (ativo em nível baixo)

    if (current_state != *last_state) {
        if (current_time - last_time > DEBOUNCE_DELAY_MS) {
            last_time = current_time;
            *last_state = current_state;
            return current_state;
        }
    }
    return *last_state;
}

void interacao_joystick_led_vermelho(){

    if (vry_value == 2047) {
        pwm_set_gpio_level(LED_VERMELHO, 0); // Desliga o LED quando o joystick está no centro
    } else{
        if (vry_value < 2047){

            uint duty_cycle_led_vermelho = 2047 - vry_value; // Calcula o duty cycle invertido (quanto mais distante do centro, mais brilho)
            pwm_set_gpio_level(LED_VERMELHO, duty_cycle_led_vermelho); // Define o duty cycle para o LED vermelho
            pwm_set_enabled(pwm_gpio_to_slice_num(LED_VERMELHO), estado_led_vermelho); // Ativa o PWM

        }else {

            uint duty_cycle_led_vermelho = vry_value - 2047; // Calcula o duty cycle invertido (quanto mais distante do centro, mais brilho)
            pwm_set_gpio_level(LED_VERMELHO, duty_cycle_led_vermelho); // Define o duty cycle para o LED vermelho
            pwm_set_enabled(pwm_gpio_to_slice_num(LED_VERMELHO), estado_led_vermelho); // Ativa o PWM
    }
}
}

void interacao_joystick_led_azul(){

    if (vrx_value == 2047) {
        pwm_set_gpio_level(LED_AZUL, 0); // Desliga o LED quando o joystick está no centro
    } else {
        if (vrx_value < 2047){

            uint duty_cycle_led_azul = 2047 - vrx_value; // Calcula o duty cycle invertido (quanto mais distante do centro, mais brilho)
            pwm_set_gpio_level(LED_AZUL, duty_cycle_led_azul); // Define o duty cycle para o LED azul
            pwm_set_enabled(pwm_gpio_to_slice_num(LED_AZUL), estado_led_azul); // Ativa o PWM

        }else{

            uint duty_cycle_led_azul = vrx_value-2047; // Calcula o duty cycle invertido (quanto mais distante do centro, mais brilho)
            pwm_set_gpio_level(LED_AZUL, duty_cycle_led_azul); // Define o duty cycle para o LED azul
            pwm_set_enabled(pwm_gpio_to_slice_num(LED_AZUL), estado_led_azul); // Ativa o PWM
        }
        
    }
}

void desenhar_bordas(ssd1306_t *ssd) {
    switch (border_style) {
        case 0:
            // borda simples
            ssd1306_rect(ssd, 0, 0, WIDTH, HEIGHT, true, false);
            break;
        case 1:
            // Borda tracejada
            for (int i = 0; i < WIDTH; i += 4) {
                ssd1306_pixel(ssd, i, 0, true);
                ssd1306_pixel(ssd, i, HEIGHT - 1, true);
            }
            for (int i = 0; i < HEIGHT; i += 4) {
                ssd1306_pixel(ssd, 0, i, true);
                ssd1306_pixel(ssd, WIDTH - 1, i, true);
            }
            break;
    }
    ssd1306_send_data(ssd);
}

// Função de interrupção para os botões
void gpio_irq_handler(uint gpio, uint32_t events);

int main() {
    // Inicializa a comunicação serial para permitir o uso de printf
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(2000);

    // Inicializa o módulo ADC do Raspberry Pi Pico
    adc_init();
    adc_gpio_init(VRX_PIN); // Configura GP26 (ADC0) para o eixo X do joystick
    adc_gpio_init(VRY_PIN); // Configura GP27 (ADC1) para o eixo Y do joystick

    // Configura o pino do botão do joystick como entrada digital com pull-up interno
    gpio_init(BOTAO_JOYSTICK);
    gpio_set_dir(BOTAO_JOYSTICK, GPIO_IN);
    gpio_pull_up(BOTAO_JOYSTICK); // Habilita o pull-up interno para garantir leitura estável

    // Configura os botões A e B como entrada digital com pull-up interno
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A); // Habilita o pull-up interno para garantir leitura estável

    // Inicializa os pinos dos LEDs como saída e desliga-os inicialmente
    gpio_init(LED_VERMELHO);
    gpio_set_dir(LED_VERMELHO, GPIO_OUT);
    gpio_put(LED_VERMELHO, false);

    gpio_init(LED_AZUL);
    gpio_set_dir(LED_AZUL, GPIO_OUT);
    gpio_put(LED_AZUL, false);

    gpio_init(LED_VERDE);
    gpio_set_dir(LED_VERDE, GPIO_OUT);
    gpio_put(LED_VERDE, false);

    // Configuração do I2C para o display ssd
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o display ssd
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Configura interrupções para os botões
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_JOYSTICK, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Inicializa o PWM para os LEDs azul e verde
    uint pwm_wrap = 4095;
    
    float DIVISOR = 10.0; 

    uint32_t last_print_time = 0; // Variável para controlar o tempo de impressão na serial

    gpio_set_function(LED_VERMELHO, GPIO_FUNC_PWM);
    gpio_set_function(LED_AZUL, GPIO_FUNC_PWM);

    // Obtém os slices PWM dos pinos
    uint slice_led_azul = pwm_gpio_to_slice_num(LED_AZUL);
    uint slice_led_vermelho = pwm_gpio_to_slice_num(LED_VERMELHO);

    // Configura o clock do PWM
    pwm_set_clkdiv(slice_led_azul, DIVISOR);
    pwm_set_clkdiv(slice_led_vermelho, DIVISOR);

    // Define o wrap para o PWM
    pwm_set_wrap(slice_led_azul, pwm_wrap);
    pwm_set_wrap(slice_led_vermelho, pwm_wrap);

    bool cor = true;

    desenhar_bordas(&ssd); // desenha a borda inicial

    while (true) {

        // Leitura do valor do ADC para VRX (Eixo X do joystick)
        adc_select_input(0); // Seleciona canal 0 (GP26 - VRX)
        vrx_value = adc_read(); // Lê o valor do eixo X, de 0 a 4095

        // Leitura do valor do ADC para VRY (Eixo Y do joystick)
        adc_select_input(1); // Seleciona canal 1 (GP27 - VRY)
        vry_value = adc_read(); // Lê o valor do eixo Y, de 0 a 4095

        // Leitura do estado do botão do joystick (SW)
        bool sw_value = gpio_get(BOTAO_JOYSTICK) == 0; // 0 indica que o botão está pressionado      

        // Verifica o estado dos botões com debounce
        bool button_a_pressed = debounce(BOTAO_A, &button_a_state); // Verifica se o botão A foi pressionado com debounce

        // Interação com os LEDs com base no movimento do joystick
        interacao_joystick_led_azul();
        interacao_joystick_led_vermelho();
    
        pos_x = MAP_x(vrx_value, 0, 4095, 3, HEIGHT - 8); // Mapeia para a largura da tela (WIDTH)
        pos_y = MAP_y(vry_value, 0, 4095, 3, WIDTH - 8);  // Mapeia para a altura da tela (HEIGHT)

        // em caso de mudança de posição desenha o quadrado 8x8 na nova posição
        if (pos_x != last_pos_x || pos_y != last_pos_y || flag_display) {
            last_pos_x = pos_x;
            last_pos_y = pos_y;

            ssd1306_fill(&ssd, false);
            ssd1306_rect(&ssd, pos_x, pos_y, 8, 8, true, true);
            desenhar_bordas(&ssd); // Atualizar a borda
        }

        // Imprime os valores lidos e o duty cycle proporcional na comunicação serial a cada 1 segundo
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_print_time >= 1000) {
            printf("VRX: %u, VRY: %u, SW: %d\n", vrx_value, vry_value, sw_value);
            printf("posição x: %d, posição y: %d\n", pos_x, pos_y);
            last_print_time = current_time; // Atualiza o tempo da última impressão
        }
    }

    return 0;
}

// função de interrupção quando um dos botões forem precionados
void gpio_irq_handler(uint gpio, uint32_t events) {

    static uint32_t last_time = 0;
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    // Debouncing de 300ms
    if (current_time - last_time > 300000) {
        last_time = current_time;

        // verifica se o botão A foi precionado
        if (gpio == BOTAO_A && !gpio_get(BOTAO_A)) {

            estado_led_azul = !estado_led_azul;
            estado_led_vermelho = !estado_led_vermelho;
            
        }
        // verifica se o botão B foi precionado
        if (gpio == BOTAO_JOYSTICK && !gpio_get(BOTAO_JOYSTICK)) {

            // inverte o estado do led verde, 
            estado_led_verde = !estado_led_verde;

            // apaga ou acente o led verde
            gpio_put(LED_VERDE, estado_led_verde);

            // mudar borda
            border_style += 1;

            if (border_style > 1){

                border_style = 0;

            }

            desenhar_bordas(&ssd); // Atualizar a borda
            
        }  

    }
}