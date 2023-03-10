# Robô Autônomo :robot:

Neste projeto foi desenvolvido um robô capaz de se locomover de forma autônoma, por meio do processamento dos dados de interação sensorial com o ambiente, através da comunicação serial entre o ROS e o Arduino. O robô utilizou para detecção de obstáculos sensores ultrassônicos e a velocidade de uma de suas rodas pôde ser lida por meio de um sensor de velocidade. Os dados obtidos pelos sensores foram acessados de forma remota, enquanto o robô navegava pelo ambiente.

### :desktop_computer: Pré-requisitos

Para executar este projeto é necessário ter instalado algumas bibliotecas e frameworks. O projeto foi desenvolvido, testado e implementado no:

- Ubuntu 18.04

- ROS Melodic

- Arduino IDE

É necessário instalar no computador o Ubuntu e o ROS, e instalar no Raspberry Pi o Ubuntu, o ROS e a IDE do Arduino. É interessante ter a mesma versão do sistema operacional e do ROS em ambos, para que não tenham problemas com compatibilidade.

#### :mag_right: Dependências

Para realizar a comunicação serial entre o ROS e o Arduino utilizou-se o pacote *rosserial_arduino*. Sua instalação pode ser feita rodando os seguintes comandos no terminal do Ubuntu:

```
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
```

```
sudo apt-get install ros-${ROS_DISTRO}-rosserial
```

Foram utilizadas algumas bibliotecas para escrever o script. Essas bibliotecas fornecem funcionalidade extra e tornam o código mais simples. Bibliotecas utilizadas:

- *Ultrasonic.h* » biblioteca para os sensores ultrassônicos

- *AFMotor.h* » biblioteca para a motor shield

#### :computer: Habilitação do SSH

Tendo como finalidade acessar as informações dos sensores utilizados no robô enquanto o mesmo circula pelo ambiente, foi-se utilizado o Secure Shell, também chamado de SSH, um protocolo de rede que permite acessar e administrar de forma remota servidores.

Para realizar o acesso remoto foi necessário habilitar o SSH nas configurações do Raspberry e pegar o seu endereço IP por meio do comando. Os comandos necessários estão escritos abaixo:

Comando para acessar as configurações do Raspberry e habilitar o SSH:

```
sudo raspi-config
```

Comando para pegar o endereço IP do Raspberry:

```
 ifconfig
```

Comando a ser inserido no terminal do computador que irá ser utilizado durante o acesso remoto, no qual {user} é o nome do usuário root e {host} é o endereço IP do Raspberry:

```
ssh {user}@{host}
```
  
### :wrench: Montagem do robô 

### :toolbox: Material utilizado 

- 1 x Arduino MEGA 2560
- 1 x Motor Shield L293D
- 4 x Motor JGB37-520 DC12V333RPM
- 3 x Sensor Ultrassônico HC-SR04
- 1 x Módulo Encoder Acoplador Óptico
- 1 x Disco Encoder
- 1 x Raspberry Pi 4
- 1 x Powerbank
- 1 x Fonte Externa de 12V
- 1 x Botão on/off
- 1 x Chassi
- 4 x Mecanum Wheels
- 1 x Computador para acesso remoto ao robô

A parte mecânica do robô é constituída por uma placa metálica com furos para fixação de componentes, quatro mecanum wheels e quatro motores  JGB37-520 DC12V333RPM, cada um com uma roda acoplada.

Levando em consideração o número de componentes a serem utilizados no projeto, foi-se escolhido o Arduino MEGA 2560, um modelo com 54 pinos digitais e 16 pinos analógicos. Seu microcontrolador é o ATmega2560, com clock de 16MHz, 256KB de memória Flash, 8KB de RAM e 4 KB de EEPROM.

Para o controle dos motores DC foi utilizada a Motor Shield L293D, um módulo que apresenta slots para conexão direta com a placa do Arduino. Esse módulo permite o controle de 2 Servos, 2 motores de passo ou o acionamento de até 4 motores DC, além de possibilitar o controle individual de cada motor.

Após conectar a shield à placa do Arduino, foi realizada a ligação com os motores DC e com uma fonte externa de 12V para a alimentação do módulo e do arduino, também foi utilizado um interruptor, responsável por ligar e desligar a alimentação.

![Untitled Diagram drawio](https://user-images.githubusercontent.com/108027884/224192878-23a8c6f2-65d0-458e-8809-e2813813fe2e.png)

Ao decorrer do projeto foram acrescentados três sensores ultrassônicos à placa do Arduino, um localizado na frente do robô, um na parte esquerda e outro na parte direita, além de um sensor de velocidade e um disco encoder feito em uma impressora 3D, utilizados para medir a rotação do motor M3.

![sensores](https://user-images.githubusercontent.com/108027884/224193508-bc4ad1bd-a6c4-4d4e-bbe3-e7280e65f47b.jpg)

![IMG-20220224-WA0055](https://user-images.githubusercontent.com/108027884/224193161-c4e17247-7e14-4700-a02d-9f8adf2e807b.jpeg)

Por fim, para ser possível a comunicação via USB entre o Raspberry Pi 4  e o Arduino MEGA 2560, adicionou-se uma base metálica ao robô, na qual foi usada para posicionar o Raspberry e uma Power Bank de 12V para sua alimentação.

![20220225_171350](https://user-images.githubusercontent.com/108027884/224191454-f43afbc0-9a0c-495b-b19a-76f633c998f0.jpg)


### :compass: Lógica de movimentação 

O algoritmo foi desenvolvido a partir de funções declaradas que definem a movimentação do robô de acordo com os dados obtidos por meio dos sensores, seguindo a lógica presente no fluxograma abaixo:

   ![Fluxograma drawio](https://user-images.githubusercontent.com/108027884/224190799-2f68cf79-1f02-48f9-8ce8-bc4d039941b2.png) 

### :movie_camera: Vídeo com os resultados obtidos 

https://www.youtube.com/watch?v=7Vnkp2oUJuo&t=216s
