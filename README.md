Projeto: Controle de LEDs RGB e Exibição de Posição do Joystick no Display SSD1306

Descrição:
Este projeto tem como objetivo consolidar os conceitos sobre o uso de conversores analógico-digitais (ADC) no microcontrolador RP2040, utilizando a placa de desenvolvimento BitDogLab. O projeto envolve o controle da intensidade de LEDs RGB com base nos valores analógicos fornecidos por um joystick, além da exibição da posição do joystick em um display SSD1306 por meio de um quadrado móvel.

O joystick fornece valores analógicos para os eixos X e Y, que são utilizados para controlar o brilho dos LEDs RGB e mover um quadrado no display. Adicionalmente, o botão do joystick e o botão A têm funcionalidades específicas, como alternar o estado do LED Verde e ativar/desativar os LEDs PWM.

Objetivos:
Compreender o funcionamento do conversor analógico-digital (ADC) no RP2040.

Utilizar o PWM para controlar a intensidade de dois LEDs RGB com base nos valores do joystick.

Representar a posição do joystick no display SSD1306 por meio de um quadrado móvel.

Aplicar o protocolo de comunicação I2C na integração com o display.

Componentes Utilizados:

BitDogLab (placa de desenvolvimento com RP2040)
LED RGB (conectado às GPIOs 11, 12 e 13)
Joystick (conectado aos GPIOs 26 e 27 para os eixos X e Y)
Botão do Joystick (conectado à GPIO 22)
Botão A (conectado à GPIO 5)
Display SSD1306 (conectado via I2C, GPIOs 14 e 15)

Funcionamento:

Controle dos LEDs RGB:
LED Azul: O brilho é ajustado conforme o valor do eixo Y do joystick. Quando o joystick está na posição central (valor 2048), o LED permanece apagado. Movendo o joystick para cima (valores menores) ou para baixo (valores maiores), o brilho do LED aumenta gradualmente, atingindo a intensidade máxima nos extremos (0 e 4095).

LED Vermelho: Segue o mesmo princípio do LED Azul, mas é controlado pelo eixo X do joystick. Movendo o joystick para a esquerda (valores menores) ou para a direita (valores maiores), o brilho do LED aumenta.

LED Verde: O estado do LED Verde é alternado a cada pressionamento do botão do joystick.

Exibição no Display SSD1306:
Um quadrado de 8x8 pixels é exibido no display, inicialmente centralizado. O quadrado se move proporcionalmente aos valores capturados pelo joystick nos eixos X e Y.

Funcionalidades Adicionais
Botão do Joystick: Alterna o estado do LED Verde e modifica o estilo da borda do display a cada pressionamento.

Botão A: Ativa ou desativa os LEDs controlados por PWM.

Autor: Mateus Moreira da Silva

Este projeto foi desenvolvido como parte da atividade prática da disciplina de programação de microcontroladores.

Recursos
Repositório: [Link para o repositório](https://github.com/Mateus-MDS/Conversor_ADC.git)

Vídeo de Demonstração: [Link para o vídeo](https://youtube.com/shorts/9M5QDpoUrlo?feature=share)
