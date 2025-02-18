Interfaces de comunicação serial na BitDogLab

Descrição:

Este projeto implementa o controle de LEDs e a exibição de caracteres na matriz de LEDs e no display da BitDogLab. A lógica do sistema permite a alternância de estados de LEDs e a exibição de letras (A-Z, a-z) e números (0-9) tanto no display quanto na matriz de LEDs.

Os estados dos LEDs seguem uma lógica específica, alternando conforme a entrada dos botões, enquanto os caracteres são processados e exibidos corretamente nos dispositivos de saída.

Componentes Necessários:

BitDogLab (com matriz de LEDs, display e botões)
Microcontrolador RP2040
Bibliotecas para controle do display e da matriz de LEDs

Funcionamento:

No controle dos LEDs, quando os botões são pressionados, os LEDs alternam entre os estados definidos.
O estado dos LEDs é atualizado no display, e também é enviado para o serial monitor.

Na exibição de Caracteres, o sistema reconhece e processa caracteres alfanuméricos (A-Z, a-z, 0-9).
O caractere atual é exibido no display OLED, e para os números tem exibição na matriz de LEDs.

Interrupções e Debouncing:

O código implementa uma lógica de interrupção para responder rapidamente à entrada dos botões.
Um mecanismo de debouncing evita múltiplos acionamentos acidentais.

Estrutura do Código:

Inicialização: Configura os pinos GPIO para LEDs e botões.
Interrupções: Monitora os botões e altera o estado dos LEDs e dos caracteres exibidos.
Renderização: Atualiza o display e a matriz de LEDs com as informações atuais.

Autor:
Mateus Moreira da Silva

Este projeto foi desenvolvido e testado utilizando a BitDogLab com o microcontrolador RP2040.
